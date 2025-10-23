#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/uart.h>

LOG_MODULE_REGISTER(cmain, CONFIG_LOG_DEFAULT_LEVEL);

/* ---------------- HW / threads ---------------- */
#define UART_NODE DT_NODELABEL(uart1)
#define STACKSIZE 1024
#define THREAD0_PRIORITY 7
#define THREAD1_PRIORITY 7
#define THREAD2_PRIORITY 7
#define THREADx_PRIORITY 3
#define THREADp_PRIORITY 5

/* ---------------- Queues ---------------- */
K_MSGQ_DEFINE(q_thread0, 1, 16, 1);
K_MSGQ_DEFINE(q_thread1, 1, 16, 1);
K_MSGQ_DEFINE(q_thread2, 1, 16, 1);

/* ---------------- Frame type ---------------- */
typedef union { uint8_t b[4]; uint32_t w; } frame_u;

/* ---- Producer <-> TX thread ---- */
K_MUTEX_DEFINE(prod_mtx);
static atomic_t prod_word = ATOMIC_INIT(0);

K_SEM_DEFINE(cmd_ready, 0, 1); /* posted by BLE write; coalesces to 1 */
static atomic_t cmd_word;      /* latest 32-bit command */

/* ---- TX thread <-> ISR ---- */
static uint8_t tx_buf[4];
K_SEM_DEFINE(sem_threadx, 0, 1);  /* ISR -> threadx: 'you may send now' */
K_SEM_DEFINE(data_ready, 0, 1);   /* max=1 so signals coalesce */

/* UART */
static const struct device *uart_dev = DEVICE_DT_GET(UART_NODE);

/* ---------------- LIN state ---------------- */
typedef enum {
    LIN_IDLE,
    LIN_SYNC,
    LIN_MSG_0,
    LIN_MSG_1,
    LIN_MSG_2,
    LIN_MSG_x
} lin_state_t;

static lin_state_t lin_state = LIN_SYNC;

/* ---------------- UART ISR ---------------- */
static void uart_cb(const struct device *dev, void *user_data)
{
    ARG_UNUSED(user_data);
    uint8_t b;

    while (uart_irq_update(dev) && uart_irq_is_pending(dev)) 
	{
        /* ---- Errors ---- */
        int err = uart_err_check(dev);
        if (err) 
		{
            if (err & UART_ERROR_OVERRUN) 
			{
                while (uart_irq_rx_ready(dev)) 
				{
                    int n = uart_fifo_read(dev, &b, 1);
                    if (n != 1) break;
                }
            }
            if (err & UART_BREAK) 
			{
                while (uart_irq_rx_ready(dev)) 
				{
                    int n = uart_fifo_read(dev, &b, 1);
                    if (n != 1) break;
                    if (b == 0x55) 
					{
                        lin_state = LIN_SYNC;
                        break;
                    }
                }
            }
        }

        /* ---- TX: drain staged 4 bytes ---- */
        if (uart_irq_tx_ready(dev)) 
		{
            (void)uart_fifo_fill(dev, &tx_buf[0], 4);
            uart_irq_tx_disable(dev);
        }

        /* ---- RX: state machine ---- */
        if (uart_irq_rx_ready(dev)) 
		{
            for (;;) 
			{
                int n = uart_fifo_read(dev, &b, 1);
                if (n != 1) break;

                if (lin_state == LIN_SYNC) 
				{
                    if (b == 0xFF)
					{
                        lin_state = LIN_MSG_0;
                    } 
					else if (b == 0x01) 
					{
                        lin_state = LIN_IDLE;
                        /* >>> Signal TX thread it MAY send now <<< */
                        k_sem_give(&sem_threadx);  /* ISR-safe */
                    } 
					else if (b == 0xF0) 
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
                    (void)k_msgq_put(&q_thread0, &b, K_NO_WAIT);
                } 
				else if (lin_state == LIN_MSG_1) 
				{
                    (void)k_msgq_put(&q_thread1, &b, K_NO_WAIT);
                } 
				else if (lin_state == LIN_MSG_2) 
				{
                    (void)k_msgq_put(&q_thread2, &b, K_NO_WAIT);
                }
            }
        }
    }
}

/* ---------------- Worker threads ---------------- */
static void thread0(void *a, void *b, void *c)
{
    ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
    uint8_t byte;
    for (;;) {
        k_msgq_get(&q_thread0, &byte, K_FOREVER);
        LOG_DBG("thread0 got: 0x%02x", byte);
    }
}

static void thread1(void *a, void *b, void *c)
{
    ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
    uint8_t byte;
    for (;;) {
        k_msgq_get(&q_thread1, &byte, K_FOREVER);
        LOG_DBG("thread1 got: 0x%02x", byte);
    }
}

static void thread2(void *a, void *b, void *c)
{
    ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
    uint8_t byte;
    for (;;) {
        k_msgq_get(&q_thread2, &byte, K_FOREVER);
        LOG_DBG("thread2 got: 0x%02x", byte);
    }
}

/* ---------------- TX thread ----------------
 * Send only when: (1) producer published NEW data (data_ready),
 * and (2) ISR granted a send slot (sem_threadx).
 */
static void threadx(void *a, void *b, void *c)
{
    ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);

    for (;;) {
        /* 1) Wait for grant from ISR */
        k_sem_take(&sem_threadx, K_FOREVER);

        /* 2) Only send if data is available *right now* */
        int rc = k_sem_take(&data_ready, K_NO_WAIT);
		if (rc != 0)
		{
			continue;   /* coalesced, already sent latest */
		}

		/* 3) Snapshot latest atomically and stage TX bytes */
        frame_u local;
        local.w = (uint32_t)atomic_get(&prod_word);

        tx_buf[0] = local.b[0];
        tx_buf[1] = local.b[1];
        tx_buf[2] = local.b[2];
        tx_buf[3] = local.b[3];

        uart_irq_tx_enable(uart_dev);

        LOG_DBG("threadx sent: %02x %02x %02x %02x",
                local.b[0], local.b[1], local.b[2], local.b[3]);
    }
}

/* ---------------- Producer thread (no dependency on consumer) ----------------
 * Can be replaced by a button ISR, timer, etc. It never blocks on TX.
 */
static void threadp(void *a, void *b, void *c)
{
    ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);

    for (;;) {
        /* Wait until a new BLE command arrives (coalesced to 1) */
        k_sem_take(&cmd_ready, K_FOREVER);

        uint32_t cmd = (uint32_t)atomic_get(&cmd_word);   /* latest wins */

        /* Hand off to your existing TX path */
        atomic_set(&prod_word, (atomic_val_t)cmd);
        k_sem_give(&data_ready);   /* wake TX worker */
    }
}

K_THREAD_DEFINE(thread0_id, STACKSIZE, thread0, NULL, NULL, NULL,
		THREAD0_PRIORITY, 0, 0);
K_THREAD_DEFINE(thread1_id, STACKSIZE, thread1, NULL, NULL, NULL,
		THREAD1_PRIORITY, 0, 0);
K_THREAD_DEFINE(thread2_id, STACKSIZE, thread2, NULL, NULL, NULL,
		THREAD2_PRIORITY, 0, 0);
K_THREAD_DEFINE(threadx_id, STACKSIZE, threadx, NULL, NULL, NULL,
		THREADx_PRIORITY, 0, 0);
K_THREAD_DEFINE(threadp_id, STACKSIZE, threadp, NULL, NULL, NULL,
		THREADp_PRIORITY, 0, 0);


/* BLE setup */
#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <string.h>

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

/* -------- Minimal connectable legacy advertising (no name) -------- */
static const struct bt_le_adv_param *adv_param = BT_LE_ADV_PARAM(
    (BT_LE_ADV_OPT_CONN | BT_LE_ADV_OPT_USE_IDENTITY),
    800, 801, NULL);

/* Keep ADV tiny: only Flags */
static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};
static const struct bt_data sd[] = { /* empty scan response */ };

/* -------- Your 128-bit UUIDs (service + single write characteristic) -------- */
/* Service: 12345678-1234-5678-1234-56789abcdef0 */
/* Char  : 12345678-1234-5678-1234-56789abcdef1 */
#define BT_UUID_CTL_SVC \
    BT_UUID_DECLARE_128(BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef0))
#define BT_UUID_CTL_CMD \
    BT_UUID_DECLARE_128(BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef1))

/* Last received 32-bit command (little-endian from air) */
static volatile uint32_t g_cmd32;

/* Write handler: accept exactly 4 bytes, offset 0 only */
static ssize_t cmd_write(struct bt_conn *conn,
                         const struct bt_gatt_attr *attr,
                         const void *buf, uint16_t len,
                         uint16_t offset, uint8_t flags)
{
    ARG_UNUSED(conn); ARG_UNUSED(attr); ARG_UNUSED(flags);

    if (offset != 0) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }
    if (len != 4) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    uint32_t v;
    memcpy(&v, buf, 4);                          /* little-endian from air */
    atomic_set(&cmd_word, (atomic_val_t)v);      /* publish latest */
    k_sem_give(&cmd_ready);                      /* wake threadp */
    return len;
}

/* Primary service: one write-only characteristic */
BT_GATT_SERVICE_DEFINE(ble_ctl_svc,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_CTL_SVC),

    BT_GATT_CHARACTERISTIC(BT_UUID_CTL_CMD,
        BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
        BT_GATT_PERM_WRITE,
        NULL,              /* no read */
        cmd_write,         /* write handler */
        NULL)              /* user data */
);

/* -------- Advertising helpers -------- */
static struct k_work adv_work;

static void adv_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);
    int err = bt_le_adv_start(adv_param, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
    if (err == -EALREADY) 
	{
        return; /* already advertising */
    }
    if (err) 
	{
        LOG_ERR("bt_le_adv_start failed: %d", err);
        return;
    }
    LOG_INF("Advertising started");
}

static void advertising_start(void)
{
    k_work_submit(&adv_work);
}

/* -------- Connection callbacks -------- */
static void on_connected(struct bt_conn *conn, uint8_t err)
{
    if (err) {
        LOG_WRN("Connection failed (err %u)", err);
        /* Ensure we keep advertising if connect failed */
        advertising_start();
        return;
    }
    LOG_INF("Connected");
}

static void on_disconnected(struct bt_conn *conn, uint8_t reason)
{
    LOG_INF("Disconnected (reason 0x%02X)", reason);
    /* Restart advertising so a new central can connect */
    advertising_start();
}

static struct bt_conn_cb conn_cbs = {
    .connected = on_connected,
    .disconnected = on_disconnected,
};

/* main */
int main(void)
{
	int err;

	if (!device_is_ready(uart_dev)) 
	{
		LOG_ERR("UART device not ready");
		return -1;
	}
	
	uart_irq_callback_user_data_set(uart_dev, uart_cb, NULL);
    
	/* Enable error + RX interrupts */
	uart_irq_rx_enable(uart_dev);
    uart_irq_err_enable(uart_dev);

	k_work_init(&adv_work, adv_work_handler);

	err = bt_enable(NULL);
	if (err) 
	{
		LOG_ERR("Bluetooth init failed (err %d)\n", err);
		return -1;
	}
	LOG_INF("Bluetooth initialized\n");
	bt_conn_cb_register(&conn_cbs);
    advertising_start();

	while(1)
	{
		k_sleep(K_FOREVER);
	}

    return 0;
}
