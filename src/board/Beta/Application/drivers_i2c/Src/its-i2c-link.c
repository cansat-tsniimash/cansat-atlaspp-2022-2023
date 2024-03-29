#include "../Inc/its-i2c-link.h"

#include <errno.h>
#include <stdio.h>
#include <assert.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

#include <stm32f0xx_ll_dma.h>
#include <stm32f0xx_ll_i2c.h>
#include <stm32f0xx_ll_gpio.h>


//#define I2C_LINK_DEBUG(...) do { printf(__VA_ARGS__); } while (0)
#define I2C_LINK_DEBUG(...) do { if (0) printf(__VA_ARGS__); } while (0)

typedef uint16_t its_i2c_link_packet_size_t;

//! Буфер для пакета (а может быть и для нескольких?
typedef struct its_i2c_link_pbuf_t
{
	its_i2c_link_packet_size_t packet_size;
	uint8_t packet_data[I2C_LINK_MAX_PACKET_SIZE];
} its_i2c_link_pbuf_t;


//! Очередь пакетов - циклический буфер
typedef struct i2c_link_pbuf_queue_t
{
	//! начало базового линейного буфера
	its_i2c_link_pbuf_t *begin;
	//! Конец базового линейного буфера
	its_i2c_link_pbuf_t *end;
	//! Голова циклобуфера
	its_i2c_link_pbuf_t *head;
	//! Хвост циклобуфера
	its_i2c_link_pbuf_t *tail;

	//! Флаг заполненности
	/*! Для разрешения неоднозначности ситуации head == tail */
	int full;
} i2c_link_pbuf_queue_t;


typedef enum i2c_link_state_dir_t
{
	I2C_LINK_STATE_DIR_RX = 0x10,
	I2C_LINK_STATE_DIR_TX = 0x20,
} i2c_link_state_dir_t;

//! Состояние i2c линка
typedef enum i2c_link_state_t
{
	//! Покой
	I2C_LINK_STATE_IDLE			= 0x00,

	//! Мы принимаем команду
	I2C_LINK_STATE_RX_CMD		= 0x01 | I2C_LINK_STATE_DIR_RX,
	//! Мы принимаем пакет
	I2C_LINK_STATE_RX_PACKET	= 0x02 | I2C_LINK_STATE_DIR_RX,
	//! Мы выбрасываем данные
	I2C_LINK_STATE_RX_DROP		= 0x03 | I2C_LINK_STATE_DIR_RX,

	//! Мы передаем размер пакета
	I2C_LINK_STATE_TX_PACKET_SIZE	= 0x01 | I2C_LINK_STATE_DIR_TX,
	//! Мы передаем сам пакет
	I2C_LINK_STATE_TX_PACKET		= 0x02 | I2C_LINK_STATE_DIR_TX,
	//! Мы отправляем нули
	I2C_LINK_STATE_TX_ZEROES		= 0x03 | I2C_LINK_STATE_DIR_TX,

} its_i2c_link_state_t;


typedef enum i2c_link_cmd_t
{
	I2C_LINK_CMD_NONE = 0x00,
	I2C_LINK_CMD_GET_SIZE = 0x01,
	I2C_LINK_CMD_GET_PACKET = 0x02,
	I2C_LINK_CMD_SET_PACKET = 0x04,
} its_i2c_link_cmd_t;


struct i2c_link_ctx_t;
typedef struct i2c_link_ctx_t i2c_link_ctx_t;


//! Контекст модуля
struct i2c_link_ctx_t
{
	//! Линейный набор приёмных буферов
	its_i2c_link_pbuf_t rx_bufs[I2C_LINK_RXBUF_COUNT];
	//! очередь линейных буферов
	i2c_link_pbuf_queue_t rx_bufs_queue;

	//! Линейный набор передаточных буферов
	its_i2c_link_pbuf_t tx_bufs[I2C_LINK_TXBUF_COUNT];
	//! очередь передаточных буферов
	i2c_link_pbuf_queue_t tx_bufs_queue;

	//! Состояние модуля
	its_i2c_link_state_t state;

	//! Выполняемая команда
	its_i2c_link_cmd_t cur_cmd;
	//! Выполняемая команда
	its_i2c_link_cmd_t prev_cmd;
	//! Буфер для приёма команды
	/*! Сделан специально чуток побольше, чтобы можно было определить,
		когда хост дает больше данных чем надо */
	uint8_t cmd_buf[15];

	//! Статистика модуля (для телеметрии в основном)
	its_i2c_link_stats_t stats;

	//Интерфейс, для которого используется i2c_link
	I2C_TypeDef *bus;

	DMA_TypeDef *dma;
	uint32_t dma_channel_tx;
	uint32_t dma_channel_rx;
	GPIO_TypeDef *sda_port;
	GPIO_TypeDef *scl_port;
	uint32_t sda_pin;
	uint32_t scl_pin;
	uint32_t afio_no;
	uint32_t irq_no;
	uint32_t irq_prio;
	uint16_t slave_addr;
};


//! Пока что мы поддерживаем ровно один i2c линк и поэтом его состояние
//! в одной глобальной переменной

static i2c_link_ctx_t _ctx;


static its_i2c_link_pbuf_t* _pbuf_queue_get_head(i2c_link_pbuf_queue_t *queue)
{
	if (queue->full)
		return 0;

	return queue->head;
}


static int _pbuf_queue_is_empty(i2c_link_pbuf_queue_t *queue)
{
	return (queue->head == queue->tail) && !queue->full;
}


static void _pbuf_queue_push_head(i2c_link_pbuf_queue_t *queue)
{
	assert(!queue->full);

	its_i2c_link_pbuf_t *next_head = queue->head + 1;
	if (next_head == queue->end)
		next_head = queue->begin;

	if (next_head == queue->tail)
		queue->full = 1;

	queue->head = next_head;
}


static its_i2c_link_pbuf_t* _pbuf_queue_get_tail(i2c_link_pbuf_queue_t *queue)
{
	if (queue->tail == queue->head && !queue->full)
		return 0;

	return queue->tail;
}


static void _pbuf_queue_pop_tail(i2c_link_pbuf_queue_t *queue)
{
	assert(!(queue->head == queue->tail && !queue->full));

	its_i2c_link_pbuf_t *next_tail = queue->tail + 1;
	if (next_tail == queue->end)
		next_tail = queue->begin;

	queue->tail = next_tail;
	queue->full = 0;
}


static int _ctx_construct(i2c_link_ctx_t *ctx)
{
	memset(ctx, 0, sizeof(*ctx));

	ctx->rx_bufs_queue.begin = ctx->rx_bufs_queue.head =
			ctx->rx_bufs_queue.tail = ctx->rx_bufs;

	ctx->rx_bufs_queue.end = ctx->rx_bufs + I2C_LINK_RXBUF_COUNT;

	ctx->tx_bufs_queue.begin = ctx->tx_bufs_queue.head =
			ctx->tx_bufs_queue.tail = ctx->tx_bufs;

	ctx->tx_bufs_queue.end = ctx->tx_bufs + I2C_LINK_TXBUF_COUNT;
	ctx->bus = I2C_LINK_BUS_HANDLE;
	ctx->dma = I2C_LINK_DMA_HANDLE;
	ctx->dma_channel_tx = I2C_LINK_DMA_CHANNEL_TX;
	ctx->dma_channel_rx = I2C_LINK_DMA_CHANNEL_RX;
	ctx->sda_port = I2C_LINK_SDA_PORT;
	ctx->scl_port = I2C_LINK_SCL_PORT;
	ctx->sda_pin = I2C_LINK_SDA_PIN;
	ctx->scl_pin = I2C_LINK_SCL_PIN;
	ctx->afio_no = I2C_LINK_PINS_AF;
	ctx->irq_no = I2C_LINK_IRQn;
	ctx->irq_prio = I2C_LINK_IRQ_PRIORITY;
	ctx->slave_addr = I2C_LINK_ADDR;
	return 0;
}


static int _bus_gpio_disable(i2c_link_ctx_t * ctx)
{
	LL_GPIO_InitTypeDef init;
	LL_GPIO_StructInit(&init);

	init.Alternate = LL_GPIO_AF_0;
	init.Mode = LL_GPIO_MODE_INPUT;

	init.Pin = ctx->sda_pin;
	LL_GPIO_Init(ctx->sda_port, &init);

	init.Pin = ctx->scl_pin;
	LL_GPIO_Init(ctx->scl_port, &init);

	return 0;
}


static int _bus_gpio_enable(i2c_link_ctx_t * ctx)
{
	LL_GPIO_InitTypeDef init;
	LL_GPIO_StructInit(&init);

	init.Mode = LL_GPIO_MODE_ALTERNATE;
	init.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
	init.Pull = LL_GPIO_PULL_NO;
	init.Alternate = I2C_LINK_PINS_AF;
	init.Speed = LL_GPIO_SPEED_FREQ_HIGH;

	init.Pin = ctx->sda_pin;
	LL_GPIO_Init(ctx->sda_port, &init);

	init.Pin = ctx->scl_pin;
	LL_GPIO_Init(ctx->scl_port, &init);

	return 0;
}


static int _bus_configure(i2c_link_ctx_t * ctx)
{
	DMA_TypeDef * const dma = ctx->dma;
	I2C_TypeDef * const bus = ctx->bus;
	const uint32_t channel_rx = ctx->dma_channel_rx;
	const uint32_t channel_tx = ctx->dma_channel_tx;

	// Настраиваем ДМА

	/* I2C1_RX Init */
	LL_DMA_SetChannelPriorityLevel(dma, channel_rx, LL_DMA_PRIORITY_HIGH);
	LL_DMA_SetDataTransferDirection(dma, channel_rx, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
	LL_DMA_SetMode(dma, channel_rx, LL_DMA_MODE_NORMAL);
	LL_DMA_SetPeriphIncMode(dma, channel_rx, LL_DMA_PERIPH_NOINCREMENT);
	LL_DMA_SetMemoryIncMode(dma, channel_rx, LL_DMA_MEMORY_INCREMENT);
	LL_DMA_SetPeriphSize(dma, channel_rx, LL_DMA_PDATAALIGN_BYTE);
	LL_DMA_SetMemorySize(dma, channel_rx, LL_DMA_MDATAALIGN_BYTE);

	/* I2C1_TX Init */
	LL_DMA_SetChannelPriorityLevel(dma, channel_tx, LL_DMA_PRIORITY_HIGH);
	LL_DMA_SetDataTransferDirection(dma, channel_tx, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
	LL_DMA_SetMode(dma, channel_tx, LL_DMA_MODE_NORMAL);
	LL_DMA_SetPeriphIncMode(dma, channel_tx, LL_DMA_PERIPH_NOINCREMENT);
	LL_DMA_SetMemoryIncMode(dma, channel_tx, LL_DMA_MEMORY_INCREMENT);
	LL_DMA_SetPeriphSize(dma, channel_tx, LL_DMA_PDATAALIGN_BYTE);
	LL_DMA_SetMemorySize(dma, channel_tx, LL_DMA_MDATAALIGN_BYTE);

	// Настраиваем I2C периферию
	LL_I2C_InitTypeDef init;
	init.PeripheralMode = LL_I2C_MODE_I2C;
	init.OwnAddress1 = ctx->slave_addr << 1;
	init.TypeAcknowledge = LL_I2C_ACK;
	init.OwnAddrSize = LL_I2C_OWNADDRESS1_7BIT;
	init.Timing = 0x2000090E; // FIXME
	init.AnalogFilter = LL_I2C_ANALOGFILTER_ENABLE; // FIXME
	init.DigitalFilter = 0; // FIXME
	LL_I2C_Init(bus, &init);
	//LL_I2C_EnableAutoEndMode(bus); // FIXME
	LL_I2C_SetOwnAddress2(bus, 0, LL_I2C_OWNADDRESS2_NOMASK);
	LL_I2C_DisableOwnAddress2(bus);
	LL_I2C_DisableGeneralCall(bus);
	LL_I2C_EnableClockStretching(bus);
	LL_I2C_DisableSlaveByteControl(bus);

	// Включаем все прерывания на уровне периферии
	LL_I2C_EnableIT_ADDR(ctx->bus);
	LL_I2C_EnableIT_ERR(ctx->bus);
	LL_I2C_EnableIT_NACK(ctx->bus);
	LL_I2C_EnableIT_STOP(ctx->bus);
	LL_I2C_EnableIT_TC(ctx->bus);
	// Это обслуживается через дма
	//LL_I2C_EnableIT_RX(ctx->bus);
	//LL_I2C_EnableIT_TX(ctx->bus);

	// в NVIC настоим только приоритет. Включим прерывания потом
	NVIC_SetPriority(ctx->irq_no, ctx->irq_prio);
	//NVIC_EnableIRQ(ctx->irq_no);

	return 0;
}


static int _bus_shutdown(i2c_link_ctx_t * ctx)
{
	if (!LL_I2C_IsEnabled(ctx->bus))
		return 0; // Уже все выключено, уходим

	// выключаем прерывания
	NVIC_DisableIRQ(ctx->irq_no);

	//! Вырубаем периферию
	LL_I2C_Disable(ctx->bus);

	// Уходим с шины, отключаем соответствующие пины
	_bus_gpio_disable(ctx);

	// Все. пока что на этом все
	return 0;
}


// Вызывается в случае, если шина ушла в резет
// из-за ошибки на ней
static int _bus_resume(i2c_link_ctx_t * ctx)
{
	if (LL_I2C_IsEnabled(ctx->bus))
		return 0; // Уже запущено

	// Все переконфигурируем, начинаем с пинов
	int rc = _bus_gpio_enable(ctx);
	if (0 != rc)
		return rc;

	rc = _bus_configure(ctx);
	if (0 != rc)
		return rc;

	// сбрасываем состояние
	ctx->cur_cmd = I2C_LINK_CMD_NONE;
	ctx->state = I2C_LINK_STATE_IDLE;

	// Включаем
	LL_I2C_Enable(ctx->bus);
	// Включаем прерывания
	NVIC_EnableIRQ(ctx->irq_no);

	ctx->stats.restarts_cnt++;
	return 0;
}


static int _link_tx_setup_for_zeroes(i2c_link_ctx_t *ctx)
{
	DMA_TypeDef * const dma = ctx->dma;
	const uint32_t channel = ctx->dma_channel_tx;

	static const uint8_t fallback[10] = { 0x00 };

	// Запускаем Дма в циклическом режиме на выдачу этих дуракцих нулей
	// из единственного байта в памяти
	LL_DMA_SetMemoryAddress(dma, channel, (uint32_t)&fallback);
	LL_DMA_SetDataLength(dma, channel, sizeof(fallback));
	LL_DMA_SetMode(dma, channel, LL_DMA_MODE_CIRCULAR);

	ctx->state = I2C_LINK_STATE_TX_ZEROES;

	ctx->stats.tx_zeroes_start_cnt++;
	I2C_LINK_DEBUG("tx 0 begun\n");
	return 0;
}


static int _link_tx_setup_for_packet_size(i2c_link_ctx_t * ctx)
{
	DMA_TypeDef * const dma = ctx->dma;
	const uint32_t channel = ctx->dma_channel_tx;

	// Мастер на шине что-то от нас хочет получить
	// Нужно посмотреть что у нас лежит в очереди на отправку
	its_i2c_link_pbuf_t *buf = _pbuf_queue_get_tail(&ctx->tx_bufs_queue);
	if (0 == buf)
	{
		// У нас ничего нет для отправки
		// нужно отправить нули...
		static const its_i2c_link_packet_size_t zero_packet_size = 0x00;

		LL_DMA_SetMemoryAddress(dma, channel, (uint32_t)&zero_packet_size);
		LL_DMA_SetDataLength(dma, channel, sizeof(zero_packet_size));
		I2C_LINK_DEBUG("tx psize 0 begun\n");

		ctx->stats.tx_empty_buffer_cnt++;
		return 0;
	}

	// буфер все-таки есть, отправляем честно сколько там места есть
	LL_DMA_SetMemoryAddress(dma, channel, (uint32_t)&buf->packet_size);
	LL_DMA_SetDataLength(dma, channel, sizeof(buf->packet_size));
	// Опять как и везде, циклический режим
	LL_DMA_SetMode(dma, channel, LL_DMA_MODE_CIRCULAR);

	ctx->state = I2C_LINK_STATE_TX_PACKET_SIZE;

	ctx->stats.tx_psize_start_cnt++;
	I2C_LINK_DEBUG("tx psize %d begun\n", buf->packet_size);
	return 0;
}


// Отправляет пакет, если он есть, и нули, если его нет
static int _link_tx_setup_for_packet(i2c_link_ctx_t *ctx)
{
	DMA_TypeDef * const dma = ctx->dma;
	const uint32_t channel = ctx->dma_channel_tx;

	// Мастер на шине что-то от нас хочет получить
	// Нужно посмотреть что у нас лежит в очереди на отправку
	its_i2c_link_pbuf_t *buf = _pbuf_queue_get_tail(&ctx->tx_bufs_queue);
	if (0 == buf)
	{
		// У нас ничего нет для отправки
		// Будем отправлять нолики из фолбека
		return _link_tx_setup_for_zeroes(ctx);
	}

	// Здесь действуем странным образом.
	// Запускаем ДМА на циклический буфер
	// И даем не только лишь данные пакета, а весь буфер пакета
	// который под него есть.
	// Делаем это на случай, если хост попросит больше данных
	// чем у нас есть. Мы. конечно, дадим хосту невалидные данные
	// но, зато сами останемся в валидном состоянии...
	LL_DMA_SetMemoryAddress(dma, channel, (uint32_t)buf->packet_data);
	LL_DMA_SetDataLength(dma, channel, I2C_LINK_MAX_PACKET_SIZE);
	LL_DMA_SetMode(dma, channel, LL_DMA_MODE_CIRCULAR);

	ctx->state = I2C_LINK_STATE_TX_PACKET;

	ctx->stats.tx_packet_start_cnt++;
	I2C_LINK_DEBUG("tx pkt begun\n");
	return 0;
}


/*
 * Обработка команды, полученной раннее
 */
static int _link_tx_dispatch(i2c_link_ctx_t * ctx)
{
	int rc;
	I2C_TypeDef * const bus = ctx->bus;
	DMA_TypeDef * const dma = ctx->dma;
	const uint32_t channel = ctx->dma_channel_tx;

	// Готовим ДМА по серьезному
	if (LL_DMA_IsEnabledChannel(dma, channel))
	{
		LL_DMA_DisableChannel(dma, channel);
		// Оно не сразу может остановится, если чем-то занято другим
		// Поэтому подождем
		// Таймауты ставить не будем, тут и так все сложно. Надеемся на вотчдоги
		// TODO: Все же сделать таймауты....
		while(LL_DMA_IsEnabledChannel(dma, channel)) {};
	}

	// Смотрим что именно будем передавать
	switch (ctx->cur_cmd)
	{
	default:
	case I2C_LINK_CMD_NONE:
		// Будем гнать нули, однозначно
		rc = _link_tx_setup_for_zeroes(ctx);
		break;

	case I2C_LINK_CMD_GET_SIZE:
		// Будет исполнено
		rc = _link_tx_setup_for_packet_size(ctx);
		break;

	case I2C_LINK_CMD_GET_PACKET:
		rc = _link_tx_setup_for_packet(ctx);
		break;
	};

	if (0 != rc)
		return rc;

	// передаем!
	LL_DMA_SetPeriphAddress(dma, channel, LL_I2C_DMA_GetRegAddr(bus, LL_I2C_DMA_REG_DATA_TRANSMIT));
	LL_DMA_EnableChannel(dma, channel);
	LL_I2C_EnableDMAReq_TX(bus);
	return 0;
}


static int _link_tx_done(i2c_link_ctx_t * ctx)
{
	I2C_TypeDef * const bus = ctx->bus;
	DMA_TypeDef * const dma = ctx->dma;
	const uint32_t channel = ctx->dma_channel_tx;

	// Выключаем ДМА
	LL_I2C_DisableDMAReq_TX(bus);
	LL_DMA_DisableChannel(dma, channel);
	while(LL_DMA_IsEnabledChannel(dma, channel))
	{
		// Таймаутов нет, только вотчдог, только так
	}

	// Чистим TX регистр
	LL_I2C_ClearFlag_TXE(bus);

	// смотрим в каком мы там были состоянии
	switch (ctx->state)
	{
	case I2C_LINK_STATE_TX_PACKET:
		// Мы пытались передать пакет, нужно скинуть его из очереди
		assert(!_pbuf_queue_is_empty(&ctx->tx_bufs_queue));
		_pbuf_queue_pop_tail(&ctx->tx_bufs_queue);
		I2C_LINK_DEBUG("tx pkt done\n");
		ctx->stats.tx_packet_done_cnt++;
		break;

	case I2C_LINK_STATE_TX_PACKET_SIZE: {
		const uint32_t dma_leftovers = LL_DMA_GetDataLength(dma, channel) + 1;
		// Добавляем 1, так как судя по всему DMA успевает пихнуть дополнительный
		// байт на TXE событие, не думая о том, что предыдущий байт был последним в транзакции
		// Так же, сравнимаем тут не с нулем, а с reload размером транзакции
		// из-за циклического режима работы ДМА
		if (dma_leftovers != sizeof(its_i2c_link_packet_size_t))
		{
			// Хост запросил неверное количество байт
			ctx->stats.tx_wrong_size_cnt++;
		}
		I2C_LINK_DEBUG("tx psize done\n");
		ctx->stats.tx_psize_done_cnt++;
		} break;

	case I2C_LINK_STATE_TX_ZEROES:
		I2C_LINK_DEBUG("tx 0 done\n");
		ctx->stats.tx_zeroes_done_cnt++;
		break;

	default:
		// А вот этого быть не должно
		abort();
	}

	// Показываем что мы тут все
	ctx->state = I2C_LINK_STATE_IDLE;
	// Команда отработана
	ctx->cur_cmd = I2C_LINK_CMD_NONE;
	return 0;
}


static int _link_rx_setup_for_drop(i2c_link_ctx_t * ctx)
{
	DMA_TypeDef * const dma = ctx->dma;
	const uint32_t channel = ctx->dma_channel_rx;

	// Будем бросать данные сюда
	static uint8_t dump;

	// Настраиваем дма в циклический режим для сброса данных
	// В этот несчастный байт
	LL_DMA_SetMemoryAddress(dma, channel, (uint32_t)&dump);
	LL_DMA_SetDataLength(dma, channel, sizeof(dump));
	LL_DMA_SetMode(dma, channel, LL_DMA_MODE_CIRCULAR);

	ctx->state = I2C_LINK_STATE_RX_DROP;

	ctx->stats.rx_drops_start_cnt++;
	I2C_LINK_DEBUG("rx drop begun\n");
	return 0;
}


static int _link_rx_setup_for_command(i2c_link_ctx_t * ctx)
{
	DMA_TypeDef * const dma = ctx->dma;
	const uint32_t channel = ctx->dma_channel_rx;

	LL_DMA_SetMemoryAddress(dma, channel, (uint32_t)&ctx->cmd_buf);
	LL_DMA_SetDataLength(dma, channel, sizeof(ctx->cmd_buf));
	LL_DMA_SetMode(dma, channel, LL_DMA_MODE_CIRCULAR);

	ctx->state = I2C_LINK_STATE_RX_CMD;

	ctx->stats.rx_cmds_start_cnt++;
	I2C_LINK_DEBUG("rx cmd begun\n");
	return 0;
}


static int _link_rx_setup_for_packet(i2c_link_ctx_t * ctx)
{
	DMA_TypeDef * const dma = ctx->dma;
	const uint32_t channel = ctx->dma_channel_rx;

	// У нас есть буфер для приёма?
	its_i2c_link_pbuf_t * head = _pbuf_queue_get_head(&ctx->rx_bufs_queue);
	if (!head)
	{
		// Нет, у нас нет такого буфера :(
		// Придется выкидывать принимаемые данные
		return _link_rx_setup_for_drop(ctx);
	}

	// Буфер есть, настраиваем на него дма
	// По аналогии с TX закрутим дма на циклический режим,
	// чтобы ни при каких условиях не останавливать шину
	LL_DMA_SetMemoryAddress(dma, channel, (uint32_t)head->packet_data);
	LL_DMA_SetDataLength(dma, channel, I2C_LINK_MAX_PACKET_SIZE);
	LL_DMA_SetMode(dma, channel, LL_DMA_MODE_CIRCULAR);

	ctx->state = I2C_LINK_STATE_RX_PACKET;

	I2C_LINK_DEBUG("rx pkt begun\n");
	ctx->stats.rx_packet_start_cnt++;
	return 0;
}


static int _link_rx_dispatch(i2c_link_ctx_t * ctx)
{
	int rc;
	I2C_TypeDef * const bus = ctx->bus;
	DMA_TypeDef * const dma = ctx->dma;
	const uint32_t channel = ctx->dma_channel_rx;

	// Готовим ДМА по серьезному
	if (LL_DMA_IsEnabledChannel(dma, channel))
	{
		LL_DMA_DisableChannel(dma, channel);
		while (LL_DMA_IsEnabledChannel(dma, channel))
		{}
	}

	switch (ctx->cur_cmd)
	{
	default:
	case I2C_LINK_CMD_NONE:
		// Слушаем команду
		rc = _link_rx_setup_for_command(ctx);
		break;

	case I2C_LINK_CMD_SET_PACKET:
		// Слушаем пакет
		rc = _link_rx_setup_for_packet(ctx);
		break;
	};

	if (0 != rc)
		return rc;

	// принимаем
	LL_DMA_SetPeriphAddress(dma, channel, LL_I2C_DMA_GetRegAddr(bus, LL_I2C_DMA_REG_DATA_RECEIVE));
	LL_DMA_EnableChannel(dma, channel);
	LL_I2C_EnableDMAReq_RX(bus);
	return 0;
}


static int _link_rx_done(i2c_link_ctx_t * ctx)
{
	I2C_TypeDef * const bus = ctx->bus;
	DMA_TypeDef * const dma = ctx->dma;
	const uint32_t channel = ctx->dma_channel_rx;

	// Выключаем ДМА
	LL_I2C_DisableDMAReq_RX(bus);
	LL_DMA_DisableChannel(dma, channel);
	while(LL_DMA_IsEnabledChannel(dma, channel))
	{}

	// Смотрим в каком мы были состоянии
	switch (ctx->state)
	{
	case I2C_LINK_STATE_RX_PACKET: {
		// Закидываем полученный пакет в очередь
		its_i2c_link_pbuf_t * const head = _pbuf_queue_get_head(&ctx->rx_bufs_queue);
		// Раз уж мы начинали писать куда-то пакет, это место долнжо быть живо
		assert(head);

		const uint32_t dma_transferred = I2C_LINK_MAX_PACKET_SIZE - LL_DMA_GetDataLength(dma, channel);
		head->packet_size =  dma_transferred;
		_pbuf_queue_push_head(&ctx->rx_bufs_queue);

		I2C_LINK_DEBUG("rx pkt done\n");
		ctx->stats.rx_packet_done_cnt++;
	} break;

	case I2C_LINK_STATE_RX_CMD: {
		const uint32_t dma_transferred = sizeof(ctx->cmd_buf) - LL_DMA_GetDataLength(dma, channel);
		// Счетчик дма должен быть нулем
		if (1 == dma_transferred) // Команды ровно по 1 байту
		{
			// Закидываем команду в "активный слот"
			ctx->cur_cmd = ctx->cmd_buf[0];
			switch (ctx->cur_cmd)
			{
			case I2C_LINK_CMD_GET_PACKET: ctx->stats.cmds_get_packet_cnt++; break;
			case I2C_LINK_CMD_GET_SIZE: ctx->stats.cmds_get_size_cnt++; break;
			case I2C_LINK_CMD_SET_PACKET: ctx->stats.cmds_set_packet_cnt++; break;
			default: ctx->stats.cmds_invalid_cnt++; break;
			};
		}
		else
		{
			// Хост передал какое-то не такое количество байт
			ctx->stats.rx_wrong_size_cnt++;
		}

		I2C_LINK_DEBUG("rx cmd set %d\n", (int)ctx->cur_cmd);
		ctx->stats.rx_cmds_done_cnt++;
	} break;

	case I2C_LINK_STATE_RX_DROP: {
		I2C_LINK_DEBUG("rx drop done\n");
		ctx->stats.rx_drops_done_cnt++;
	} break;

	default:
		abort();
	}

	if (I2C_LINK_STATE_RX_CMD != ctx->state)
	{
		// Если мы только что не получили команду, то скидываем её
		// как выполненую
		ctx->cur_cmd = I2C_LINK_CMD_NONE;
	}

	ctx->state = I2C_LINK_STATE_IDLE;
	return 0;
}


int its_i2c_link_start()
{
	i2c_link_ctx_t *ctx = &_ctx;

	int rc = _ctx_construct(ctx);
	if (0 != rc)
		return rc;

	ctx->state = I2C_LINK_STATE_IDLE;

	// Настраиваем периферию и запускаемся
	rc = _bus_resume(ctx);
	if (0 != rc)
		return rc;

	return 0;
}


int its_i2c_link_reset(void)
{
	// Вот так грязно рубим все
	_bus_shutdown(&_ctx);

	return 0;
}


int its_i2c_link_write(const void *data, size_t data_size)
{
	i2c_link_ctx_t *const ctx = &_ctx;
	_bus_resume(ctx);

	if (data_size > I2C_LINK_MAX_PACKET_SIZE)
		return -EINVAL;

	its_i2c_link_pbuf_t *buf = _pbuf_queue_get_head(&ctx->tx_bufs_queue);
	if (0 == buf)
	{
		ctx->stats.tx_overruns_cnt++;
		return -EAGAIN;
	}

	buf->packet_size = data_size;
	memcpy(buf->packet_data, data, data_size);
	memset(buf->packet_data + data_size, 0x00, sizeof(buf->packet_data) - data_size);
	_pbuf_queue_push_head(&ctx->tx_bufs_queue);

	return data_size;
}


int its_i2c_link_read(void *buffer_, size_t buffer_size)
{
	i2c_link_ctx_t *const ctx = &_ctx;
	_bus_resume(ctx);

	its_i2c_link_pbuf_t *buf = _pbuf_queue_get_tail(&ctx->rx_bufs_queue);
	if (0 == buf)
		return -EAGAIN;

	uint8_t *data = (uint8_t*) buffer_;
	size_t portion_size;
	if (buffer_size < buf->packet_size)
		portion_size = buffer_size;
	else
		portion_size = buf->packet_size;

	memcpy(data, buf->packet_data, portion_size);
	int packet_size = buf->packet_size;

	// Очистим буфер, чтобы в следующем проходе в нем не было мусора
	memset(buf->packet_data, 0, I2C_LINK_MAX_PACKET_SIZE);
	_pbuf_queue_pop_tail(&ctx->rx_bufs_queue);

	return packet_size;
}


void its_i2c_link_stats(its_i2c_link_stats_t *statsbuf)
{
	i2c_link_ctx_t *const ctx = &_ctx;

	*statsbuf = ctx->stats;
}


static int _link_event_handler(i2c_link_ctx_t * ctx)
{
	int rc = 0;

	const volatile uint32_t isr = ctx->bus->ISR;

	I2C_LINK_DEBUG("evt 0x%04lX\n", isr);

	if (isr & I2C_ISR_STOPF)
	{
		LL_I2C_ClearFlag_STOP(ctx->bus);
		I2C_LINK_DEBUG("stopf\n");

		// поидее этот флаг может стрелять всегда и он будет означать конец транзакции..
		// смотрим по нашему состоянию
		if (ctx->state & I2C_LINK_STATE_DIR_RX)
			rc = _link_rx_done(ctx);
		else if (ctx->state & I2C_LINK_STATE_DIR_TX)
			rc = _link_tx_done(ctx);
		else if (ctx->state == I2C_LINK_STATE_IDLE)
		{
			// Вообще это странно, но если уж мы в идле, то
			// не будем ничего делать
		}
		else
		{
			// Это состояние очень уже похоже на невалидное
			// TODO: запросить сброс перефирии
		}
	}

	if (isr & I2C_ISR_NACKF)
	{
		LL_I2C_ClearFlag_NACK(ctx->bus);
		// AF мы поидее не можем поймать нигде кроме как на TX транзакции
		// И это будет означать конец этой самой транзакции.
		// После этого AF должен бы пойти stopf
		// по чип работает хитро и почему-то его нет...
		I2C_LINK_DEBUG("af\n");
		if (ctx->state & I2C_LINK_STATE_DIR_TX)
			rc = _link_tx_done(ctx);
		else
			// Если мы не в TX, но случилась такая лажа...
			ctx->stats.af_cnt++;
	}

	if (isr & I2C_ISR_ARLO)
	{
		LL_I2C_ClearFlag_ARLO(ctx->bus);
		_bus_shutdown(ctx);
		// Вполне может быть
		// Вырубаемся и ждем переиницилизации
		I2C_LINK_DEBUG("arlo\n");
		ctx->stats.arlo_cnt++;
	}

	if (isr & I2C_ISR_BERR)
	{
		// Прямо очень часто бывает
		// Так же тут работает еррата о том, что модуль после этого никак вообще не может работать
		// Вырубаемся и ждем переиницилизации
		LL_I2C_ClearFlag_BERR(ctx->bus);
		_bus_shutdown(ctx);

		I2C_LINK_DEBUG("berr\n");
		ctx->stats.berr_cnt++;
	}

	if (isr & I2C_ISR_OVR)
	{
		// Вот этого вообще-то быть не должно
		// Мы затягиваем клоки и работаем на дма
		// Едвали мы что-то не успеем
		LL_I2C_ClearFlag_OVR(ctx->bus);
		_bus_shutdown(ctx);

		I2C_LINK_DEBUG("ovr\n");
		ctx->stats.ovf_cnt++;
	}

	if (isr & I2C_ISR_TC)
	{
		// Этот бит зажигается, если игнорировать RXNE или TXNE
		// за нас RXNE и TXNE должно обслуживать DMA
		// Если оно не работает...
		// Надо все рубить
		LL_I2C_ClearFlag_BERR(ctx->bus); // fixme
		_bus_shutdown(ctx);

		I2C_LINK_DEBUG("btf\n");
		ctx->stats.btf_cnt++;
	}

	if (isr & I2C_ISR_PECERR || isr & I2C_ISR_TIMEOUT || isr & I2C_ISR_ALERT)
	{
		// Вот этого НИКОГДА быть не должно
		// так это функционал smbus

		// Грузимся по жесткому
		abort();
	}


	if (isr & I2C_ISR_ADDR)
	{
		LL_I2C_ClearFlag_ADDR(ctx->bus);
		I2C_LINK_DEBUG("addr\n");

		// Кто-то на шине назвал наш адрес!
		// нужно понять он просит нас принять данные или отдать
		if (isr & I2C_ISR_DIR)
		{
			// нужно отдавать, окей, отдаем
			rc = _link_tx_dispatch(ctx);
		}
		else
		{
			// нужно принимать. Окей, это мы тоже умеем
			rc = _link_rx_dispatch(ctx);
		}
		// Если мы не смогли адекватно отреагировать
		// на начало транзакции... Плохи наши дела
		// TODO: что-нибудь с этим сделать
		assert(0 == rc);
	}


	return 0;
}


void its_i2c_link_it_handler(void)
{
	i2c_link_ctx_t * const ctx = &_ctx;
	_link_event_handler(ctx);
}

