#include "uart_dma.h"
#include "can_link.h"
#include "inspection_app.h"
#include "byte_codec.h"
#include "usart.h"
#include "can.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static DMA_HandleTypeDef rx_dma[7];
UART_HandleTypeDef huart1={.hdmarx=&rx_dma[1],.gState=1,.tag=1};
UART_HandleTypeDef huart2={.hdmarx=&rx_dma[2],.gState=1,.tag=2};
UART_HandleTypeDef huart5={.hdmarx=&rx_dma[5],.gState=1,.tag=5};
UART_HandleTypeDef huart6={.hdmarx=&rx_dma[6],.gState=1,.tag=6};
static CAN_TypeDef can_regs;
CAN_HandleTypeDef hcan1={.Instance=&can_regs};
static uint32_t tick, next_sequence;
static unsigned tx_counts[256], can_status[4], can_writes;
static bool can_respond=true;
static struct {
    uint8_t *rx, *active;
    uint16_t size, position, length;
    uint8_t saved[UART_TX_MAX_BYTES];
    uint32_t finish;
    bool busy;
} uart_model[7];
static DmCanFrame can_fifo[4];
static unsigned can_in, can_out;
static uint16_t last_chassis_valid, last_chassis_status, last_gyro_valid;
static uint32_t telemetry_replacements;
static void (*tick_read_hook)(void);

uint32_t HAL_GetTick(void)
{
    uint32_t snapshot=tick;
    if (tick_read_hook) {
        void (*hook)(void)=tick_read_hook; tick_read_hook=NULL; hook();
    }
    return snapshot;
}
void Error_Handler(void) { abort(); }

HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *u, uint8_t *b, uint16_t n)
{
    uart_model[u->tag].rx=b; uart_model[u->tag].size=n; uart_model[u->tag].position=0;
    u->hdmarx->counter=n; return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *u, uint8_t *b, uint16_t n)
{
    unsigned tag=u->tag;
    if (uart_model[tag].busy) return HAL_BUSY;
    assert(n <= UART_TX_MAX_BYTES);
    uart_model[tag].active=b; uart_model[tag].length=n; memcpy(uart_model[tag].saved,b,n);
    uart_model[tag].finish=tick+(n*10000U+115199U)/115200U;
    uart_model[tag].busy=true; u->gState=2;
    if (tag==2) {
        assert(b[0]==0xAA && b[1]==0x55 && b[2]==1);
        assert((unsigned)n==ReadU16LE(b+6)+10U);
        assert(Host_Crc16(b+2,n-4)==ReadU16LE(b+n-2));
        assert(ReadU16LE(b+4)==(uint16_t)next_sequence++);
        ++tx_counts[b[3]];
        if (b[3]==0x10) { assert(n==50); last_chassis_valid=ReadU16LE(b+44); last_chassis_status=ReadU16LE(b+46); }
        if (b[3]==0x11) { assert(n==24); last_gyro_valid=ReadU16LE(b+20); }
        if (b[3]==0x12) { assert(n==30); telemetry_replacements=ReadU32LE(b+20); }
        if (b[3]==0x10 && tick>=980) assert((uint32_t)(tick-ReadU32LE(b+8))<=20);
    }
    return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Abort_IT(UART_HandleTypeDef *u)
{
    uart_model[u->tag].busy=false; u->gState=1;
    HAL_UART_AbortCpltCallback(u); return HAL_OK;
}
static void Advance(uint32_t now)
{
    tick=now;
    UART_HandleTypeDef *ports[]={&huart1,&huart2,&huart5,&huart6};
    for (unsigned i=0;i<4;++i) {
        UART_HandleTypeDef *u=ports[i]; unsigned tag=u->tag;
        bool stalled=tag==2 && tick>=900 && tick<980;
        if (!stalled && uart_model[tag].busy && (int32_t)(tick-uart_model[tag].finish)>=0) {
            /* DMA must still see exactly the submitted bytes at completion. */
            assert(memcmp(uart_model[tag].active,uart_model[tag].saved,uart_model[tag].length)==0);
            uart_model[tag].busy=false; u->gState=1; HAL_UART_TxCpltCallback(u);
        }
    }
}

static void Push(UART_HandleTypeDef *u, UartDma *direct, const uint8_t *data, size_t n, bool idle)
{
    unsigned tag=u->tag;
    for (size_t i=0;i<n;++i) {
        unsigned pos=uart_model[tag].position;
        uart_model[tag].rx[pos]=data[i];
        pos=(pos+1U)%uart_model[tag].size;
        uart_model[tag].position=(uint16_t)pos;
        u->hdmarx->counter=uart_model[tag].size-pos;
        if (pos==0 || pos==uart_model[tag].size/2U) {
            if (direct) UartDma_Capture(direct); else HAL_UARTEx_RxEventCallback(u,(uint16_t)pos);
        }
    }
    if (idle) {
        if (direct) UartDma_Capture(direct); else HAL_UARTEx_RxEventCallback(u,uart_model[tag].position);
    }
}

static void ReceiveOnNextMillisecond(void)
{
    /* Simulate an RX IRQ after Poll took its initial tick snapshot, but before
       Receive drains the queue. Capture uses the NEW millisecond. */
    ++tick;
    uint8_t payload[16]={0},frame[HOST_MAX_FRAME];
    WriteU32LE(payload,tick); WriteF32LE(payload+4,0.4f);
    size_t length=Host_Encode(HOST_MSG_MOTION,0,payload,16,frame);
    Push(&huart2,NULL,frame,length,true);
}

HAL_StatusTypeDef HAL_CAN_ConfigFilter(CAN_HandleTypeDef *p, CAN_FilterTypeDef *f)
{
    (void)p;
    assert(f->FilterMode==CAN_FILTERMODE_IDLIST && f->FilterScale==CAN_FILTERSCALE_16BIT);
    assert(f->FilterIdHigh==(0x11U<<5) && f->FilterMaskIdLow==(0x14U<<5));
    return HAL_OK;
}
HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef *p) { (void)p; return HAL_OK; }
HAL_StatusTypeDef HAL_CAN_ActivateNotification(CAN_HandleTypeDef *p,uint32_t mask)
{ (void)p; assert(mask&CAN_IT_BUSOFF); return HAL_OK; }
uint32_t HAL_CAN_GetTxMailboxesFreeLevel(CAN_HandleTypeDef *p) { (void)p; return 3; }
uint32_t HAL_CAN_GetError(CAN_HandleTypeDef *p) { return p->error; }
uint32_t HAL_CAN_GetRxFifoFillLevel(CAN_HandleTypeDef *p,uint32_t fifo)
{ (void)p;(void)fifo;return can_in-can_out; }
HAL_StatusTypeDef HAL_CAN_GetRxMessage(CAN_HandleTypeDef *p,uint32_t fifo,CAN_RxHeaderTypeDef *h,uint8_t *b)
{
    (void)p;(void)fifo;
    if (can_in==can_out) return HAL_ERROR;
    DmCanFrame *f=&can_fifo[can_out++%4U];
    *h=(CAN_RxHeaderTypeDef){.StdId=f->id,.DLC=f->length}; memcpy(b,f->data,8); return HAL_OK;
}
HAL_StatusTypeDef HAL_CAN_AddTxMessage(CAN_HandleTypeDef *p,CAN_TxHeaderTypeDef *h,uint8_t *b,uint32_t *mailbox)
{
    *mailbox=0;
    /* Startup must remain read-only: fail immediately on enable/velocity/write. */
    if (h->StdId!=0x7FF || (b[2]!=0x33 && b[2]!=0xCC)) { ++can_writes; assert(0); }
    assert(h->IDE==CAN_ID_STD && h->RTR==CAN_RTR_DATA && h->DLC==8);
    unsigned id=ReadU16LE(b); assert(id>=1 && id<=4);
    if (!can_respond) return HAL_OK;
    DmCanFrame f={.id=(uint16_t)(0x10U+id),.length=8};
    if (b[2]==0xCC) {
        ++can_status[id-1];
        const uint8_t state[8]={0x10,0x80,0,0xA0,2,0,40,41};
        memcpy(f.data,state,8); f.data[0]|=(uint8_t)id;
    } else {
        WriteU16LE(f.data,(uint16_t)id); f.data[2]=0x33; f.data[3]=b[3];
        switch (b[3]) {
        case 0x15:WriteF32LE(f.data+4,12.5f);break;
        case 0x16:WriteF32LE(f.data+4,40);break;
        case 0x17:WriteF32LE(f.data+4,8);break;
        case 0x06:WriteF32LE(f.data+4,35);break;
        case 0x0A:WriteU32LE(f.data+4,3);break;
        default:assert(0);
        }
    }
    assert(can_in-can_out<4); can_fifo[can_in++%4U]=f;
    HAL_CAN_RxFifo0MsgPendingCallback(p); return HAL_OK;
}

static void TestDma(void)
{
    static UartDma p; UartRxBlock block;
    assert(UartDma_Init(&p,&huart1)==HAL_OK);
    p.head=p.tail=UINT32_MAX-7U; /* Wrap software queue counters too. */
    uint8_t bytes[512]; for (unsigned i=0;i<512;++i) bytes[i]=(uint8_t)(i%251);
    for (unsigned round=0;round<8;++round) {
        /* No idle at all: continuous traffic serviced solely by HT/TC. */
        Push(&huart1,&p,bytes,sizeof(bytes),false);
        unsigned got=0;
        while (UartDma_Read(&p,&block)) {
            assert(memcmp(block.data,bytes+got,block.length)==0); got+=block.length;
        }
        assert(got==512);
        UartDma_Capture(&p); UartDma_Capture(&p); assert(!UartDma_Read(&p,&block));
    }
    for (unsigned i=0;i<UART_RX_BLOCK_COUNT+1U;++i) Push(&huart1,&p,bytes,1,true);
    assert(p.rx_overflows==1 && !UartDma_Read(&p,&block));
    assert(UartDma_TakeGap(&p)); assert(!UartDma_Read(&p,&block));
    Push(&huart1,&p,bytes,4,true); assert(UartDma_Read(&p,&block) && block.length==4);
    assert(UartDma_Transmit(&p,bytes,100)); uint8_t saved[100]; memcpy(saved,p.tx_active,100);
    memset(bytes,0,sizeof(bytes)); assert(!UartDma_Transmit(&p,bytes,100));
    assert(memcmp(saved,p.tx_active,100)==0);
    Advance(100); UartDma_OnTxComplete(&p);
    UartDma_OnError(&p); assert(UartDma_TakeGap(&p)); UartDma_Poll(&p,100);
    assert(p.recovery==2); /* Manual callback for this standalone test port. */
    UartDma_OnAbortComplete(&p); UartDma_Poll(&p,100); assert(p.recovery==0 && !p.tx_busy);
    /* A fresh error after 24.8 days must not be compared against a zero retry
       deadline as if it were in the future. Exercise tick rollover too. */
    tick=UINT32_MAX-5U; UartDma_OnError(&p); UartDma_Poll(&p,tick);
    assert(p.recovery==2); UartDma_OnAbortComplete(&p);
    UartDma_Poll(&p,3); assert(p.recovery==0);
    puts("PASS: circular DMA without IDLE, wrap, duplicate event, overflow/gap, TX ownership, async recovery");
}

static void TestCanQueue(void)
{
    CanLink p; assert(CanLink_Init(&p,&hcan1)==HAL_OK);
    for (unsigned i=0;i<CAN_RX_QUEUE_COUNT+1U;++i) {
        can_fifo[can_in++%4U]=(DmCanFrame){.id=0x11,.length=8};
        CanLink_OnRx(&p);
    }
    CanRxItem item; assert(p.overflows==1 && !CanLink_Read(&p,&item));
    assert(CanLink_TakeGap(&p) && !CanLink_Read(&p,&item));
    can_fifo[can_in++%4U]=(DmCanFrame){.id=0x12,.length=8};
    CanLink_OnRx(&p); assert(CanLink_Read(&p,&item) && item.frame.id==0x12);
    can_regs.ESR=CAN_ESR_BOFF; hcan1.error=HAL_CAN_ERROR_BOF;
    CanLink_OnError(&p); assert(CanLink_BusOff(&p) && CanLink_TakeGap(&p));
    DmCanFrame query={.id=0x7FF,.length=8}; assert(!CanLink_Send(&p,&query));
    can_regs.ESR=0; hcan1.error=0;
    puts("PASS: CAN RX overflow invalidation and bus-off send refusal");
}

static void TestApplication(void)
{
    memset(uart_model,0,sizeof(uart_model)); tick=0;
    huart1.gState=huart2.gState=huart5.gState=huart6.gState=1;
    Inspection_Init();
    MotionLimits control_limits;
    assert(!Inspection_GetMotionLimits(&control_limits));
    const float reference[4] = {0}; SwerveTargets targets;
    ChassisPlan preview;
    assert(!Inspection_GetControlPreview(&preview));
    assert(!Inspection_CalculateWheels(reference, &targets));
    for (unsigned i=0;i<4;++i) assert(Inspection_ConfigureWheelFeedback(i,1,1));
    uint8_t payload[16]={0}, frame[HOST_MAX_FRAME];
    WriteF32LE(payload+4,1); WriteF32LE(payload+8,0.2f); WriteF32LE(payload+12,0.1f);
    uint8_t w[11]={0x55,0x52,0,0,0,0,0,1,0,0,0}, a[11]={0x55,0x53,0,0,0,0,0,0x40,0,0,0};
    for (unsigned i=0;i<10;++i) {w[10]+=w[i];a[10]+=a[i];}
    for (unsigned ms=0;ms<=1200;++ms) {
        Advance(ms);
        if (ms<400 && ms%10==0) {
            WriteU32LE(payload,ms); size_t n=Host_Encode(1,(uint16_t)(ms/10),payload,16,frame);
            Push(&huart2,NULL,frame,n,true);
        }
        if (ms<600 && ms%20==0) {Push(&huart6,NULL,w,11,true);Push(&huart6,NULL,a,11,true);}
        if (ms==600) can_respond=false;
        Inspection_Poll();
        if (ms==450) {
            ChassisOutputBatch output; ChassisOutputDiagnostics diagnostics;
            assert(!Inspection_BuildOutputPreview(&output,&diagnostics));
            for (unsigned i=0;i<4;++i) {
                assert(diagnostics.wheel_errors[i]&CHASSIS_WHEEL_CALIBRATION);
                assert(Inspection_ConfigureSteering(i,i&1 ? -1 : 1));
            }
            assert(Inspection_BuildOutputPreview(&output,&diagnostics));
            for (unsigned i=0;i<4;++i) assert(output.drive[i].id==0x201+i && output.drive[i].length==4);
            assert(Inspection_GetControlPreview(&preview) && !preview.command_timeout);
            assert(Inspection_GetMotionLimits(&control_limits));
            /* Mock MAX_SPD=35rad/s, less than physical no-load speed. */
            for (unsigned i=0;i<4;++i) assert(fabsf(control_limits.wheel_speed_rad_s[i]-24.5f)<0.00002f);
            assert(fabsf(control_limits.linear_speed_m_s-1.53125f)<0.00002f);
            assert(Inspection_CalculateWheels(reference, &targets));
            for (unsigned i=0;i<4;++i) {
                const float x[4]={0.1975f,0.1975f,-0.1975f,-0.1975f};
                const float y[4]={0.1425f,-0.1425f,0.1425f,-0.1425f};
                float speed=targets.wheel[i].speed_rad_s*0.0625f;
                assert(fabsf(speed*cosf(targets.wheel[i].angle_rad)-(1-0.1f*y[i]))<0.00002f);
                assert(fabsf(speed*sinf(targets.wheel[i].angle_rad)-(0.2f+0.1f*x[i]))<0.00002f);
            }
            for (unsigned i=0;i<4;++i) {
                const Dm3519 *m=Inspection_Motor(i); assert(m->parameters==31);
                assert(m->feedback_count>=20 && Dm3519_FeedbackValid(m,ms,100));
            }
            assert(last_chassis_valid==15 && last_gyro_valid==3 && last_chassis_status==0);
        }
        if (ms==800) {
            ChassisOutputBatch output; ChassisOutputDiagnostics diagnostics;
            assert(!Inspection_BuildOutputPreview(&output,&diagnostics));
            for (unsigned i=0;i<4;++i) assert(diagnostics.wheel_errors[i]&CHASSIS_WHEEL_FEEDBACK);
            assert(Inspection_GetControlPreview(&preview) && preview.command_timeout);
            assert(!Inspection_CalculateWheels(reference, &targets));
            HostMotion motion; assert(!Inspection_GetMotion(&motion));
            assert(motion.vx==0 && motion.vy==0 && motion.wz==0);
            assert(last_gyro_valid==0 && last_chassis_valid==0 && (last_chassis_status&1));
        }
    }
    assert(tx_counts[0x10]>=55 && tx_counts[0x10]<=60);
    assert(tx_counts[0x11]>=55 && tx_counts[0x11]<=60);
    assert(telemetry_replacements>=4); /* Congestion replaces stale snapshots. */
    assert(tx_counts[0x12]==1 && can_writes==0);
    for (unsigned i=0;i<4;++i) assert(can_status[i]>=29 && can_status[i]<=30);
    HAL_UART_ErrorCallback(&huart2); Inspection_Poll(); Inspection_Poll();
    size_t n=Host_Encode(1,0,payload,16,frame);Push(&huart2,NULL,frame,n,true);
    Inspection_Poll(); HostMotion motion; assert(Inspection_GetMotion(&motion));
    Advance(1210); Inspection_Poll();
    assert(Inspection_GetControlPreview(&preview) && !preview.command_timeout);
    Advance(1240); tick_read_hook=ReceiveOnNextMillisecond; Inspection_Poll();
    assert(tick==1241 && Inspection_GetControlPreview(&preview));
    assert(!preview.command_timeout && preview.timestamp_ms==1241);
    puts("PASS: simulated 100Hz host / 50Hz gyro and motor polling, 50Hz telemetry, stale data, UART recovery, no motor writes");
}

void TestProtocols(void);
void TestKinematics(void);
void TestMotion(void);
void TestPlanner(void);
void TestOutput(void);
void TestControlModes(void);
int main(void)
{
    TestProtocols(); TestKinematics(); TestMotion(); TestPlanner(); TestOutput(); TestControlModes(); TestDma(); TestCanQueue(); TestApplication(); puts("ALL TESTS PASSED"); return 0;
}
