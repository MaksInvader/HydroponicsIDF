#include <stdio.h>
#include <string.h>
#include "unity.h"
#include "runtime_safety.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

static QueueHandle_t test_queue = NULL;
static TaskHandle_t test_dosing_task = NULL;
static TaskHandle_t test_sensor_task = NULL;
static TaskHandle_t test_comm_task = NULL;
static TaskHandle_t test_safety_task = NULL;
static atomic_bool test_stop_requested = ATOMIC_VAR_INIT(false);

void setUp(void)
{
    /* Create test queue */
    test_queue = xQueueCreate(10, sizeof(int));
    TEST_ASSERT_NOT_NULL(test_queue);

    /* Reset safety state */
    runtime_safety_reset_state();
    runtime_safety_reset_heartbeats();
}

void tearDown(void)
{
    /* Clean up test queue */
    if (test_queue != NULL) {
        vQueueDelete(test_queue);
        test_queue = NULL;
    }

    /* Unbind safety module */
    runtime_safety_bind(NULL);
}

void test_safety_module_initialization(void)
{
    runtime_safety_bindings_t bindings = {
        .dosing_queue = &test_queue,
        .dosing_task = &test_dosing_task,
        .sensor_task = &test_sensor_task,
        .comm_task = &test_comm_task,
        .safety_task = &test_safety_task,
        .stop_requested = &test_stop_requested,
        .zone_id = "test_zone",
        .zone_name = "Test Zone",
        .safety_fault_topic = "test/fault"
    };

    runtime_safety_bind(&bindings);

    /* Verify initial state */
    TEST_ASSERT_FALSE(runtime_safety_is_safe_mode());
    TEST_ASSERT_EQUAL(0, runtime_safety_get_faults());
    TEST_ASSERT_FALSE(runtime_safety_is_gate_open(false));
}

void test_safety_gate_control(void)
{
    runtime_safety_bindings_t bindings = {
        .dosing_queue = &test_queue,
        .dosing_task = &test_dosing_task,
        .sensor_task = &test_sensor_task,
        .comm_task = &test_comm_task,
        .safety_task = &test_safety_task,
        .stop_requested = &test_stop_requested,
        .zone_id = "test_zone",
        .zone_name = "Test Zone",
        .safety_fault_topic = "test/fault"
    };

    runtime_safety_bind(&bindings);

    /* Test gate open */
    runtime_safety_set_gate(true);
    TEST_ASSERT_TRUE(runtime_safety_is_gate_open(false));

    /* Test gate close */
    runtime_safety_set_gate(false);
    TEST_ASSERT_FALSE(runtime_safety_is_gate_open(false));
}

void test_safety_channel_state_tracking(void)
{
    runtime_safety_bindings_t bindings = {
        .dosing_queue = &test_queue,
        .dosing_task = &test_dosing_task,
        .sensor_task = &test_sensor_task,
        .comm_task = &test_comm_task,
        .safety_task = &test_safety_task,
        .stop_requested = &test_stop_requested,
        .zone_id = "test_zone",
        .zone_name = "Test Zone",
        .safety_fault_topic = "test/fault"
    };

    runtime_safety_bind(&bindings);

    /* Test channel state updates */
    runtime_safety_update_channel_state(ACTUATOR_CHANNEL_VALVE, true);
    TEST_ASSERT_TRUE(runtime_safety_get_channel_state(ACTUATOR_CHANNEL_VALVE));

    runtime_safety_update_channel_state(ACTUATOR_CHANNEL_VALVE, false);
    TEST_ASSERT_FALSE(runtime_safety_get_channel_state(ACTUATOR_CHANNEL_VALVE));

    /* Test copy channel states */
    bool states[ACTUATOR_CHANNEL_COUNT] = {0};
    runtime_safety_update_channel_state(ACTUATOR_CHANNEL_PER_NUTA, true);
    runtime_safety_update_channel_state(ACTUATOR_CHANNEL_PER_NUTB, true);
    
    runtime_safety_copy_channel_states(states);
    TEST_ASSERT_TRUE(states[ACTUATOR_CHANNEL_PER_NUTA]);
    TEST_ASSERT_TRUE(states[ACTUATOR_CHANNEL_PER_NUTB]);
    TEST_ASSERT_FALSE(states[ACTUATOR_CHANNEL_VALVE]);
}

void test_safety_heartbeat_tracking(void)
{
    runtime_safety_bindings_t bindings = {
        .dosing_queue = &test_queue,
        .dosing_task = &test_dosing_task,
        .sensor_task = &test_sensor_task,
        .comm_task = &test_comm_task,
        .safety_task = &test_safety_task,
        .stop_requested = &test_stop_requested,
        .zone_id = "test_zone",
        .zone_name = "Test Zone",
        .safety_fault_topic = "test/fault"
    };

    runtime_safety_bind(&bindings);

    /* Reset heartbeats */
    runtime_safety_reset_heartbeats();

    /* Increment heartbeats */
    runtime_safety_heartbeat_dosing();
    runtime_safety_heartbeat_sensor();
    runtime_safety_heartbeat_comm();
    runtime_safety_heartbeat_safety();

    /* Note: We can't directly verify heartbeat values without exposing internal state,
     * but we can verify the functions don't crash */
    TEST_PASS();
}

void test_safety_fault_management(void)
{
    runtime_safety_bindings_t bindings = {
        .dosing_queue = &test_queue,
        .dosing_task = &test_dosing_task,
        .sensor_task = &test_sensor_task,
        .comm_task = &test_comm_task,
        .safety_task = &test_safety_task,
        .stop_requested = &test_stop_requested,
        .zone_id = "test_zone",
        .zone_name = "Test Zone",
        .safety_fault_topic = "test/fault"
    };

    runtime_safety_bind(&bindings);

    /* Set a fault */
     runtime_safety_fault_set(SAFETY_FAULT_VALVE, "Test fault");
    TEST_ASSERT_NOT_EQUAL(0, runtime_safety_get_faults());
    TEST_ASSERT_TRUE(runtime_safety_is_safe_mode());

    /* Clear fault */
    bool safe_mode_cleared = false;
    esp_err_t ret = runtime_safety_clear_faults(SAFETY_FAULT_VALVE, &safe_mode_cleared);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    TEST_ASSERT_TRUE(safe_mode_cleared);
    TEST_ASSERT_EQUAL(0, runtime_safety_get_faults());
    TEST_ASSERT_FALSE(runtime_safety_is_safe_mode());
}

void test_safety_boot_faults(void)
{
    /* Set boot faults */
    runtime_safety_set_boot_faults(SAFETY_FAULT_WDT);
    TEST_ASSERT_EQUAL(SAFETY_FAULT_WDT, runtime_safety_get_boot_faults());

    /* OR additional boot faults */
    runtime_safety_or_boot_faults(SAFETY_FAULT_POWER);
    TEST_ASSERT_EQUAL(SAFETY_FAULT_WDT | SAFETY_FAULT_POWER, runtime_safety_get_boot_faults());

    /* Reset */
    runtime_safety_set_boot_faults(0);
    TEST_ASSERT_EQUAL(0, runtime_safety_get_boot_faults());
}

void app_main(void)
{
    printf("\n\n=== Runtime Safety Module Tests ===\n");
    
    UNITY_BEGIN();
    
    RUN_TEST(test_safety_module_initialization);
    RUN_TEST(test_safety_gate_control);
    RUN_TEST(test_safety_channel_state_tracking);
    RUN_TEST(test_safety_heartbeat_tracking);
    RUN_TEST(test_safety_fault_management);
    RUN_TEST(test_safety_boot_faults);
    
    UNITY_END();
    
    printf("\n=== Tests Complete ===\n\n");
}
