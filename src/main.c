#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(cmain, CONFIG_LOG_DEFAULT_LEVEL);

// Device Tree UART node
#define UART_NODE DT_NODELABEL(uart1)
// Thread definitions
#define STACKSIZE 1024
#define THREAD0_PRIORITY 7
#define THREAD1_PRIORITY 7
#define THREAD2_PRIORITY 7
// Message queue definitions
K_MSGQ_DEFINE(q_thread0, 1, 16, 1);
K_MSGQ_DEFINE(q_thread1, 1, 16, 1);
K_MSGQ_DEFINE(q_thread2, 1, 16, 1);

/* UART */
#include <zephyr/drivers/uart.h>

static const struct device *uart_dev = DEVICE_DT_GET(UART_NODE);
struct uart_config uart_cfg = 
{
	.baudrate = 19200,
	.parity = UART_CFG_PARITY_NONE,
	.stop_bits = UART_CFG_STOP_BITS_1,
	.data_bits = UART_CFG_DATA_BITS_8,
	.flow_ctrl = UART_CFG_FLOW_CTRL_NONE,
};
typedef enum { LIN_SYNC, LIN_MSG_0, LIN_MSG_1, LIN_MSG_2, LIN_MSG_x } lin_state_t;
static lin_state_t lin_state = LIN_SYNC;

static void uart_cb(const struct device *dev, void *user_data)
{
    ARG_UNUSED(user_data);
    uint8_t buf[1];

    /* Standard IRQ pump: keep going while there’s work pending */
    while (uart_irq_update(dev) && uart_irq_is_pending(dev)) 
	{
        /* --- Error handling (bitmask) ------------------------------------- */
        int err = uart_err_check(dev);  /* reads & clears latched error bits */
        if (err) 
		{
            if (err & UART_ERROR_OVERRUN) 
			{
                for (;;) 
				{
                    int n = uart_fifo_read(dev, buf, sizeof(buf));
                    LOG_DBG("OVERRUN drain read n=%d", n);
                    if (n <= 0) { break; }
                }
            }
            if (err & UART_ERROR_PARITY) 
			{
                //LOG_INF("ERR: PARITY detected");
            }
            if (err & UART_ERROR_FRAMING) 
			{
				//LOG_INF("ERR: FRAME detected");
            }
            if (err & UART_BREAK) 
			{
                for (;;) 
				{
                    int n = uart_fifo_read(dev, buf, sizeof(buf));
                    LOG_DBG("BREAK drain read n=%d", n);
                    if (n <= 0) { break; }
					if (buf[0] == 0x55)
					{
						lin_state = LIN_SYNC;
					}
                }
            }
        } 

        /* --- RX ready: pull everything available -------------------------- */
        int rx_ready = uart_irq_rx_ready(dev);
        if (rx_ready) 
		{
            for (;;) 
			{
                int n = uart_fifo_read(dev, buf, sizeof(buf));
				if (n <= 0) { break; }
				if (lin_state == LIN_SYNC)
				{
					if (buf[0] == 0xFF)
					{
						lin_state = LIN_MSG_0;
					}
					else if (buf[0] == 0x01)
					{
						lin_state = LIN_MSG_1;
					}
					else if (buf[0] == 0xF0)
					{
						lin_state = LIN_MSG_2;
					}
					else
					{
						lin_state = LIN_MSG_x;
					}
				}
				else if (lin_state == LIN_MSG_0)
				{
					k_msgq_put(&q_thread0, &buf[0], K_NO_WAIT);
				}
				else if (lin_state == LIN_MSG_1)
				{
					k_msgq_put(&q_thread1, &buf[0], K_NO_WAIT);
				}
				else if (lin_state == LIN_MSG_2)
				{
					k_msgq_put(&q_thread2, &buf[0], K_NO_WAIT);
				}
            }
        }
    }
}

/* Threads */
static void thread0(void *a, void *b, void *c)
{
    ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);

    uint8_t byte;
	for (;;) 
	{
        /* Block until a byte is posted to q_thread0 */
        k_msgq_get(&q_thread0, &byte, K_FOREVER);
        LOG_INF("thread0 got: 0x%02x", byte);
    }
}

static void thread1(void *a, void *b, void *c)
{
    ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);

    uint8_t byte;
    for (;;) 
	{
        /* Block until a byte is posted to q_thread1 */
        k_msgq_get(&q_thread1, &byte, K_FOREVER);
        LOG_INF("thread1 got: 0x%02x", byte);
    }
}

static void thread2(void *a, void *b, void *c)
{
    ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);

    uint8_t byte;
    for (;;) 
	{
        /* Block until a byte is posted to q_thread2 */
        k_msgq_get(&q_thread2, &byte, K_FOREVER);
        LOG_INF("thread2 got: 0x%02x", byte);
    }
}

K_THREAD_DEFINE(thread0_id, STACKSIZE, thread0, NULL, NULL, NULL,
		THREAD0_PRIORITY, 0, 0);
K_THREAD_DEFINE(thread1_id, STACKSIZE, thread1, NULL, NULL, NULL,
		THREAD1_PRIORITY, 0, 0);
K_THREAD_DEFINE(thread2_id, STACKSIZE, thread2, NULL, NULL, NULL,
		THREAD2_PRIORITY, 0, 0);


/* BLE setup */
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gap.h>

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, ( BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)), 					// advertising flags
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),		// advertising data
};

static const struct bt_data sd[] = {

};

static 

/* main */
int main(void)
{
	int err;

	if (!device_is_ready(uart_dev)) 
	{
		LOG_ERR("UART device not ready");
		return -1;
	}
	
	/*
	int rc = uart_configure(uart_dev, &uart_cfg);
	if (rc != 0 && rc != -ENOSYS)
	{
		LOG_ERR("uart_configure failed: %d", rc);
		return -1;
	}
	*/

	err = bt_enable(NULL);
	if (err) 
	{
		LOG_ERR("Bluetooth init failed (err %d)\n", err);
		return -1;
	}
	LOG_INF("Bluetooth initialized\n");
	
	uart_irq_callback_user_data_set(uart_dev, uart_cb, NULL);
    
	/* Enable error + RX interrupts */
	uart_irq_rx_enable(uart_dev);
    uart_irq_err_enable(uart_dev);

	while(1)
	{
		k_sleep(K_FOREVER);
	}

    return 0;
}
