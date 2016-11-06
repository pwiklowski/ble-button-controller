#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "ble.h"
#include "app_util.h"
#include "app_error.h"
#include "app_gpiote.h"
#include "app_util.h"
#include "app_timer.h"
#include "app_trace.h"
#include "ble_advdata_parser.h"
#include "ble.h"
//#include "ble_uart_c.h"
#include "ble_db_discovery.h"
#include "bsp.h"
#include "device_manager.h"
#include "nordic_common.h"
#include "nrf_sdm.h"
#include "nrf_gpio.h"
#include "pstorage.h"
#include "softdevice_handler.h"
#include "simple_uart.h"


#define SEC_PARAM_BOND             0                                  /**< Perform bonding. */
#define SEC_PARAM_MITM             0                                  /**< Man In The Middle protection not required. */
#define SEC_PARAM_IO_CAPABILITIES  BLE_GAP_IO_CAPS_NONE               /**< No I/O capabilities. */
#define SEC_PARAM_OOB              0                                  /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE     7                                  /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE     16                                 /**< Maximum encryption key size. */

#define SCAN_INTERVAL              0x0010                             /**< Determines scan interval in units of 0.625 millisecond. */
#define SCAN_WINDOW                0x0010                             /**< Determines scan window in units of 0.625 millisecond. */

#define MIN_CONNECTION_INTERVAL    MSEC_TO_UNITS(7.5, UNIT_1_25_MS)   /**< Determines maximum connection interval in millisecond. */
#define MAX_CONNECTION_INTERVAL    MSEC_TO_UNITS(30, UNIT_1_25_MS)    /**< Determines maximum connection interval in millisecond. */
#define SLAVE_LATENCY              0                                  /**< Determines slave latency in counts of connection events. */
#define SUPERVISION_TIMEOUT        MSEC_TO_UNITS(4000, UNIT_10_MS)    /**< Determines supervision time-out in units of 10 millisecond. */

#define TARGET_UUID                0x180D                             /**< Target device name that application is looking for. */
#define MAX_PEER_COUNT             DEVICE_MANAGER_MAX_CONNECTIONS     /**< Maximum number of peer's application intends to manage. */
#define UUID16_SIZE                2                                  /**< Size of 16 bit UUID */
#define BUTTON_DETECTION_DELAY               APP_TIMER_TICKS(50, APP_TIMER_PRESCALER)   /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */
#define APP_TIMER_PRESCALER                  0                                          /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_MAX_TIMERS                 4                                          /**< Maximum number of simultaneously created timers. */
#define APP_TIMER_OP_QUEUE_SIZE              5                                          /**< Size of timer operation queues. */
#define UART_SEND_INTERVAL          APP_TIMER_TICKS(1000, APP_TIMER_PRESCALER) /**< Battery level measurement interval (ticks). */
#define DEAD_BEEF                            0xDEADBEEF                                 /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */


typedef uint32_t 	ret_code_t;
/**@breif Macro to unpack 16bit unsigned UUID from octet stream. */
#define UUID16_EXTRACT(DST,SRC)                                                                  \
        do                                                                                       \
        {                                                                                        \
            (*(DST)) = (SRC)[1];                                                                 \
            (*(DST)) <<= 8;                                                                      \
            (*(DST)) |= (SRC)[0];                                                                \
        } while(0)

/**@brief Variable length data encapsulation in terms of length and pointer to data */
typedef struct
{
    uint8_t     * p_data;                                         /**< Pointer to data. */
    uint16_t      data_len;                                       /**< Length of data. */
}data_t;

typedef enum
{
    BLE_NO_SCAN,                                                  /**< No advertising running. */
    BLE_WHITELIST_SCAN,                                           /**< Advertising with whitelist. */
    BLE_FAST_SCAN,                                                /**< Fast advertising running. */
} ble_advertising_mode_t;

static ble_db_discovery_t           m_ble_db_discovery;                  /**< Structure used to identify the DB Discovery module. */
//static ble_uart_c_t                  m_ble_uart_c;                         /**< Structure used to identify the heart rate client module. */

static ble_gap_scan_params_t        m_scan_param;                        /**< Scan parameters requested for scanning and connection. */
static dm_application_instance_t    m_dm_app_id;                         /**< Application identifier. */
static dm_handle_t                  m_dm_device_handle;                  /**< Device Identifier identifier. */
static uint8_t                      m_peer_count = 0;                    /**< Number of peer's connected. */
static uint8_t                      m_scan_mode;                         /**< Scan mode used by application. */

static bool                         m_memory_access_in_progress = false; /**< Flag to keep track of ongoing operations on persistent memory. */

uint8_t   nus_service_uuid[16] = {0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
                                     0x93, 0xF3, 0xA3, 0xB5, 0x01, 0x00, 0x40, 0x6E};
/**
 * @brief Connection parameters requested for connection.
 */
static const ble_gap_conn_params_t m_connection_param =
{
    (uint16_t)MIN_CONNECTION_INTERVAL,   // Minimum connection
    (uint16_t)MAX_CONNECTION_INTERVAL,   // Maximum connection
    0,                                   // Slave latency
    (uint16_t)SUPERVISION_TIMEOUT        // Supervision time-out
};

int _write(int fd, char * str, int len)
{
    for (int i = 0; i < len; i++)
    {
        simple_uart_put(str[i]);
    }
    return len;
}

static void scan_start(void);

/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in]   line_num   Line number of the failing ASSERT call.
 * @param[in]   file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


static uint32_t adv_report_parse(uint8_t type, data_t * p_advdata, data_t * p_typedata)
{
    uint32_t index = 0;
    uint8_t * p_data;

    p_data = p_advdata->p_data;

    while (index < p_advdata->data_len)
    {
        uint8_t field_length = p_data[index];
        uint8_t field_type = p_data[index+1];

        if (field_type == type)
        {
            p_typedata->p_data = &p_data[index+2];
            p_typedata->data_len = field_length-1;
            return NRF_SUCCESS;
        }
        index += field_length+1;
    }
    return NRF_ERROR_NOT_FOUND;
}


static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t                err_code;
    const ble_gap_evt_t   * p_gap_evt = &p_ble_evt->evt.gap_evt;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_ADV_REPORT:
        {

        	//if (p_gap_evt->params.adv_report.peer_addr.addr[5] == 0x20) break;

            data_t adv_data;
            data_t type_data;
            data_t manufacturer_data;

            manufacturer_data.data_len=0;

            // Initialize advertisement report for parsing.
            adv_data.p_data = (uint8_t *)p_gap_evt->params.adv_report.data;
            adv_data.data_len = p_gap_evt->params.adv_report.dlen;
//
//            err_code = adv_report_parse(BLE_GAP_AD_TYPE_128BIT_SERVICE_UUID_MORE_AVAILABLE,
//            		&adv_data,
//					&type_data);
//
            if (adv_data.data_len == 0) break;

            err_code = adv_report_parse(BLE_GAP_AD_TYPE_MANUFACTURER_SPECIFIC_DATA,
                        		&adv_data,
            					&manufacturer_data);

//
//
            simple_uart_put('A');
			simple_uart_put('D');
			simple_uart_put('V');

            simple_uart_put(p_gap_evt->params.adv_report.peer_addr.addr[5]);
			simple_uart_put(p_gap_evt->params.adv_report.peer_addr.addr[4]);
			simple_uart_put(p_gap_evt->params.adv_report.peer_addr.addr[3]);
			simple_uart_put(p_gap_evt->params.adv_report.peer_addr.addr[2]);
			simple_uart_put(p_gap_evt->params.adv_report.peer_addr.addr[1]);
			simple_uart_put(p_gap_evt->params.adv_report.peer_addr.addr[0]);
			simple_uart_put(manufacturer_data.data_len);

			for(int i=0;i<manufacturer_data.data_len;i++)
			{
				simple_uart_put(manufacturer_data.p_data[i]);
			}

//            printf("ADV %02x%02x%02x%02x%02x%02x rssi=%d\n", p_gap_evt->params.adv_report.peer_addr.addr[5],
//            		p_gap_evt->params.adv_report.peer_addr.addr[4],
//					p_gap_evt->params.adv_report.peer_addr.addr[3],
//					p_gap_evt->params.adv_report.peer_addr.addr[2],
//					p_gap_evt->params.adv_report.peer_addr.addr[1],
//					p_gap_evt->params.adv_report.peer_addr.addr[0],
//					adv_data.p_data[ adv_data.data_len-1]);

            break;
        }
        case BLE_GAP_EVT_TIMEOUT:
            if(p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_SCAN)
            {
                if (m_scan_mode ==  BLE_WHITELIST_SCAN)
                {
                    m_scan_mode = BLE_FAST_SCAN;

                    // Start non selective scanning.
                    scan_start();
                }
            }
            else if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN)
            {
                //
            }
            break;
        case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
            // Accepting parameters requested by peer.
            err_code = sd_ble_gap_conn_param_update(p_gap_evt->conn_handle,
                                                    &p_gap_evt->params.conn_param_update_request.conn_params);
            APP_ERROR_CHECK(err_code);
            break;
        default:
            break;
    }
}

static void on_sys_evt(uint32_t sys_evt)
{
    switch(sys_evt)
    {
        case NRF_EVT_FLASH_OPERATION_SUCCESS:
        case NRF_EVT_FLASH_OPERATION_ERROR:
            if (m_memory_access_in_progress)
            {
                m_memory_access_in_progress = false;
                scan_start();
            }
            break;
        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the scheduler in the main loop after a BLE stack event has
 *  been received.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    dm_ble_evt_handler(p_ble_evt);
    ble_db_discovery_on_ble_evt(&m_ble_db_discovery, p_ble_evt);
    //ble_uart_c_on_ble_evt(&m_ble_uart_c, p_ble_evt);

    on_ble_evt(p_ble_evt);
}


/**@brief Function for dispatching a system event to interested modules.
 *
 * @details This function is called from the System event interrupt handler after a system
 *          event has been received.
 *
 * @param[in]   sys_evt   System stack event.
 */
static void sys_evt_dispatch(uint32_t sys_evt)
{
    pstorage_sys_event_handler(sys_evt);
    on_sys_evt(sys_evt);
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;

    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_RC_250_PPM_4000MS_CALIBRATION, NULL);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for System events.
    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the Device Manager.
 *
 * @details Device manager is initialized here.
 */
static void device_manager_init(void)
{
    dm_application_param_t param;
    dm_init_param_t        init_param;

    uint32_t              err_code;

    err_code = pstorage_init();
    APP_ERROR_CHECK(err_code);

//    // Clear all bonded devices if user requests to.
//    init_param.clear_persistent_data =
//        ((nrf_gpio_pin_read(BOND_DELETE_ALL_BUTTON_ID) == 0)? true: false);

    // Clear all bonded devices always
    init_param.clear_persistent_data = true;

    err_code = dm_init(&init_param);
    APP_ERROR_CHECK(err_code);

    memset(&param.sec_param, 0, sizeof (ble_gap_sec_params_t));

    // Service or protocol context for device manager to load, store and apply on behalf of application.
    // Here set to client as application is a GATT client.
    param.service_type           = DM_PROTOCOL_CNTXT_GATT_CLI_ID;

    // Secuirty parameters to be used for security procedures.
    param.sec_param.bond         = SEC_PARAM_BOND;
    param.sec_param.mitm         = SEC_PARAM_MITM;
    param.sec_param.io_caps      = SEC_PARAM_IO_CAPABILITIES;
    param.sec_param.oob          = SEC_PARAM_OOB;
    param.sec_param.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
    param.sec_param.max_key_size = SEC_PARAM_MAX_KEY_SIZE;
    param.sec_param.kdist_periph.enc = 1;
    param.sec_param.kdist_periph.id  = 1;

    err_code = dm_register(&m_dm_app_id,&param);
    APP_ERROR_CHECK(err_code);
}




/** @brief Function for the Power manager.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}



static void db_discovery_init(void)
{
    uint32_t err_code = ble_db_discovery_init();
    APP_ERROR_CHECK(err_code);
}


static void scan_start(void)
{
    uint32_t              err_code;
    uint32_t              count;

    err_code = pstorage_access_status_get(&count);
    APP_ERROR_CHECK(err_code);

    if (count != 0)
    {
        m_memory_access_in_progress = true;
        return;
    }


    APP_ERROR_CHECK(err_code);

	m_scan_param.active       = 1;            // Active scanning set.
	m_scan_param.selective    = 0;            // Selective scanning not set.
	m_scan_param.interval     = SCAN_INTERVAL;// Scan interval.
	m_scan_param.window       = SCAN_WINDOW;  // Scan window.
	m_scan_param.p_whitelist  = NULL;         // No whitelist provided.
	m_scan_param.timeout      = 0x0000;       // No timeout.

    err_code = sd_ble_gap_scan_start(&m_scan_param);
    APP_ERROR_CHECK(err_code);
}

static void timers_init(void)
{
    // Initialize timer module.
    APP_TIMER_INIT(APP_TIMER_PRESCALER, 1, 2, false);

    // Create timers.
}


int main(void)
{
    uint32_t err_code;
	simple_uart_config(RTS_PIN_NUMBER, 9, CTS_PIN_NUMBER, 8, false);

    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS, APP_TIMER_OP_QUEUE_SIZE, NULL);
    APP_GPIOTE_INIT(1);
    err_code = bsp_init(BSP_INIT_LED | BSP_INIT_BUTTONS, APP_TIMER_TICKS(100, APP_TIMER_PRESCALER),NULL);
    APP_ERROR_CHECK(err_code);
    timers_init();


    ble_stack_init();
    device_manager_init();
    db_discovery_init();

    scan_start();
    for (;;)
    {
        power_manage();
    }
}

