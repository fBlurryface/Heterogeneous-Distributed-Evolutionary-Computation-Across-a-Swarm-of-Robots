#include <Arduino.h>
#include <Wire.h>
#include "ircomm_i2c.h"

//==================== 宏定义 ====================//
#define PRINT_TX             1    // 打印发送负载
#define PRINT_RX             1    // 打印接收负载
#define PRINT_GA_LOG_EVERY   0    // 每多少代打印一次日志；0=每代
#define ELITE_COUNT          2    // （保留原 GA 宏，为兼容注入逻辑）
#define IMM_PER_GEN          1    // “移民”注入参数
#define MAXLEN               32   // I²C 单事务最大
#define RX_DRAIN_MAX         16   // 单次最大读出条数

//==================== 通用参数 ====================//
static const uint8_t  DIM             = 7;
static const uint8_t  POP_SIZE        = 10;     // ≥4
static const uint16_t MAX_GEN         = 5000;  // 保留原 GA 代数
static const float    X_MIN           = -5.12f;
static const float    X_MAX           =  5.12f;
static const uint16_t CAPTURE_INTERVAL= 30;     // 捕获间隔代数
static const uint8_t  SEG_LEN         = 3;
static const uint16_t RX_POLL_MS      = 2;      // 接收轮询间隔

//==================== DE 参数 ====================//
static const float DE_F  = 0.6f;  // 缩放系数
static const float DE_CR = 0.9f;  // 交叉概率

//==================== 全局数据区 ====================//
// DE 种群与统计
float    population[POP_SIZE][DIM];
float    obj_val[POP_SIZE];
float    fitness_val[POP_SIZE];
float    best_solution[DIM];
float    best_obj_val;
uint16_t cur_gen = 0;

// DE 临时向量
float mutant_vec[DIM];
float trial_vec[DIM];

// 接收缓冲池
#define RX_POOL_SIZE 8
struct ExtSegment {
  float segment[SEG_LEN];
  float fitness;
  bool  used;
};
ExtSegment rxPool[RX_POOL_SIZE];
uint8_t    rxHead        = 0;
uint8_t    rxTotal       = 0;
uint32_t   rx_total_msgs = 0;

// 控制开关 & 串口命令
bool    de_enabled      = false;
char    pending_cmd[16];
bool    has_pending_cmd = false;

//==================== 通用工具函数 ====================//
static inline uint32_t xorshift32(uint32_t x) {
  x ^= x << 13; x ^= x >> 17; x ^= x << 5;
  return x;
}
uint32_t quickSeedADC() {
  const uint8_t pins[] = {A0,A1,A2,A3,A4,A5,A6,A7};
  uint32_t s = micros();
  for (uint8_t p = 0; p < sizeof(pins); p++) {
    analogRead(pins[p]);
    for (uint8_t i = 0; i < 4; i++) {
      s ^= (analogRead(pins[p]) & 0x03);
      s ^= micros();
      s = xorshift32(s);
      delayMicroseconds(120);
    }
  }
  return xorshift32(s ^ micros());
}
float randFloat01()               { return random(0,10000)/10000.0f; }
float randFloat(float lo,float hi){ return lo + (hi-lo)*randFloat01(); }
float rastrigin(const float *x) {
  float s = 10.0f * DIM;
  for (uint8_t i = 0; i < DIM; i++)
    s += x[i]*x[i] - 10.0f * cos(2.0f * PI * x[i]);
  return s;
}
float calcFitness(float f)        { return 1.0f / (1.0f + f); }

static bool equalsIgnoreCase(const char *a, const char *b) {
  while (*a && *b) {
    char ca = (*a >= 'a' && *a <= 'z') ? *a - 32 : *a;
    char cb = (*b >= 'a' && *b <= 'z') ? *b - 32 : *b;
    if (ca != cb) return false;
    a++; b++;
  }
  return *a == 0 && *b == 0;
}

//==================== “移民”注入 & 最差个体查找 ====================//
static uint8_t findWorstIdx() {
  uint8_t w = 0;
  float wf = fitness_val[0];
  for (uint8_t i = 1; i < POP_SIZE; i++) {
    if (fitness_val[i] < wf) {
      wf = fitness_val[i];
      w = i;
    }
  }
  return w;
}

void injectRxSegmentsIntoPopulation() {
  if (rxTotal == 0) return;
  uint8_t injected = 0;
  for (uint8_t k = 0; k < RX_POOL_SIZE && injected < IMM_PER_GEN; k++) {
    uint8_t idx = (rxHead + RX_POOL_SIZE - 1 - k) % RX_POOL_SIZE;
    if (k >= rxTotal || rxPool[idx].used) continue;
    uint8_t victim = findWorstIdx();
    uint8_t start  = random(0, DIM - SEG_LEN + 1);
#if PRINT_RX
    Serial.print("INJ ▶ pool["); Serial.print(idx);
    Serial.print("] -> pop["); Serial.print(victim);
    Serial.print("] pos="); Serial.print(start); Serial.print(" : ");
    for (uint8_t j = 0; j < SEG_LEN; j++) {
      Serial.print(rxPool[idx].segment[j],4);
      if (j < SEG_LEN-1) Serial.print(",");
    }
    Serial.println();
#endif
    for (uint8_t j = 0; j < SEG_LEN; j++)
      population[victim][start + j] = rxPool[idx].segment[j];
    rxPool[idx].used = true;
    injected++;
    obj_val[victim]     = rastrigin(population[victim]);
    fitness_val[victim] = calcFitness(obj_val[victim]);
  }
}

//==================== I²C 简封装 ====================//
static inline bool i2cWrite1(uint8_t b) {
  Wire.beginTransmission(IRCOMM_IR_ADDR);
  Wire.write(b);
  return Wire.endTransmission() == 0;
}

//==================== 接收 & 存储逻辑 ====================//
static bool recvRaw(char *buf, uint8_t &outLen) {
  ir_mode_t m;
  for (uint8_t ch = 0; ch < 4; ch++) {
    m.mode = MODE_SIZE_MSG0 + ch;
    if (!i2cWrite1(m.mode)) return false;
    Wire.requestFrom(IRCOMM_IR_ADDR,1);
    if (Wire.available()<1) return false;
    uint8_t len = Wire.read();
    if (len==0 || len>=MAXLEN) continue;
    m.mode = MODE_REPORT_MSG0 + ch;
    if (!i2cWrite1(m.mode)) return false;
    Wire.requestFrom(IRCOMM_IR_ADDR,len);
    if (Wire.available()<len) return false;
    for (uint8_t i=0;i<len;i++) buf[i]=Wire.read();
    buf[len]=0; outLen=len;
    if (buf[0]=='!') continue;
    return true;
  }
  return false;
}

static bool recvDataChunk(float *dataOut, uint8_t maxLen, uint8_t &outCount) {
  char buf[MAXLEN]; uint8_t len;
  if (!recvRaw(buf,len) || !strchr(buf,',')) return false;
  rx_total_msgs++;
  outCount = 0;
  char *p = buf;
  while (outCount < maxLen && p) {
    dataOut[outCount++] = atof(p);
    p = strchr(p,',');
    if (p) p++;
  }
  return true;
}

static void storeRxSegment(const float seg[SEG_LEN], float fitness) {
  for (uint8_t i = 0; i < rxTotal; i++) {
    uint8_t idx = (rxHead + RX_POOL_SIZE - 1 - i) % RX_POOL_SIZE;
    if (rxPool[idx].fitness == fitness) return;
  }
  ExtSegment &slot = rxPool[rxHead];
  memcpy(slot.segment, seg, sizeof(slot.segment));
  slot.fitness = fitness;
  slot.used    = false;
  rxHead = (rxHead + 1) % RX_POOL_SIZE;
  if (rxTotal < RX_POOL_SIZE) rxTotal++;
}

void task_rx_poll() {
  static unsigned long last = 0;
  if (millis() - last < RX_POLL_MS) return;
  last = millis();
  uint8_t drained = 0;
  while (drained < RX_DRAIN_MAX) {
    float data[SEG_LEN+1]; uint8_t cnt;
    if (!(recvDataChunk(data,SEG_LEN+1,cnt) && cnt==SEG_LEN+1)) break;
    storeRxSegment(data, data[SEG_LEN]);
    drained++;
  }
}

//==================== DE 相关：演化 & 状态机 ====================//
enum DEStage {
  DE_INIT,
  DE_EVAL,
  DE_EVOLVE,
  DE_UPDATE_BEST,
  DE_CAPTURE_SEND,
  DE_LOG,
  DE_NEXTGEN,
  DE_DONE
};
static DEStage de_stage = DE_INIT;
static uint8_t  de_i    = 0;

void de_init_population() {
  for (uint8_t i = 0; i < POP_SIZE; i++)
    for (uint8_t j = 0; j < DIM; j++)
      population[i][j] = randFloat(X_MIN, X_MAX);
}

static void evaluateOne(uint8_t idx) {
  obj_val[idx]     = rastrigin(population[idx]);
  fitness_val[idx] = calcFitness(obj_val[idx]);
}

void task_de_tick() {
  if (de_stage == DE_DONE) return;
  switch (de_stage) {
    case DE_INIT: {
      uint32_t seed = quickSeedADC();
      randomSeed(seed);
      Serial.print("Seed,0x"); Serial.print(seed, HEX);
      Serial.print(","); Serial.println(seed);
      de_init_population();
      cur_gen = 0;
      rx_total_msgs = 0;   // ← 在这里清零
      de_i = 0;
      de_stage = DE_EVAL;
      break;
    }
    case DE_EVAL: {
      evaluateOne(de_i++);
      if (de_i >= POP_SIZE) {
        de_i = 0;
        de_stage = DE_EVOLVE;
      }
      break;
    }
    case DE_EVOLVE: {
      if (de_i < POP_SIZE) {
        uint8_t r1, r2, r3;
        do { r1 = random(0, POP_SIZE); } while (r1 == de_i);
        do { r2 = random(0, POP_SIZE); } while (r2 == de_i || r2 == r1);
        do { r3 = random(0, POP_SIZE); } while (r3 == de_i || r3 == r1 || r3 == r2);

        for (uint8_t j = 0; j < DIM; j++) {
          mutant_vec[j] = population[r1][j]
                         + DE_F * (population[r2][j] - population[r3][j]);
          mutant_vec[j] = constrain(mutant_vec[j], X_MIN, X_MAX);
        }

        uint8_t jrand = random(0, DIM);
        for (uint8_t j = 0; j < DIM; j++) {
          if (randFloat01() < DE_CR || j == jrand)
            trial_vec[j] = mutant_vec[j];
          else
            trial_vec[j] = population[de_i][j];
        }

        float trial_obj = rastrigin(trial_vec);
        if (trial_obj <= obj_val[de_i]) {
          memcpy(population[de_i], trial_vec, DIM * sizeof(float));
          obj_val[de_i]     = trial_obj;
          fitness_val[de_i] = calcFitness(trial_obj);
        }
        de_i++;
      } else {
        de_i = 0;
        de_stage = DE_UPDATE_BEST;
      }
      break;
    }
    case DE_UPDATE_BEST: {
      best_obj_val = obj_val[0];
      memcpy(best_solution, population[0], DIM * sizeof(float));
      for (uint8_t k = 1; k < POP_SIZE; k++) {
        if (obj_val[k] < best_obj_val) {
          best_obj_val = obj_val[k];
          memcpy(best_solution, population[k], DIM * sizeof(float));
        }
      }
      de_stage = DE_CAPTURE_SEND;
      break;
    }
    case DE_CAPTURE_SEND: {
      cur_gen++;
      if (cur_gen % CAPTURE_INTERVAL == 0) {
        // 1. 找最优个体片段
        uint8_t best_idx = 0; float best_f = obj_val[0];
        for (uint8_t i = 1; i < POP_SIZE; i++) {
          if (obj_val[i] < best_f) {
            best_f = obj_val[i];
            best_idx = i;
          }
        }
        uint8_t start = random(0, DIM - SEG_LEN + 1);
        float seg[SEG_LEN];
        for (uint8_t j = 0; j < SEG_LEN; j++) {
          seg[j] = population[best_idx][start + j];
        }
        // 2. 发送 & 3. 注入
        sendCaptureSegment(seg);
        injectRxSegmentsIntoPopulation();
      }
      de_stage = DE_LOG;
      break;
    }
    case DE_LOG: {
      if (PRINT_GA_LOG_EVERY == 0 || cur_gen % PRINT_GA_LOG_EVERY == 0) {
        float gen_best = obj_val[0];
        for (uint8_t k = 1; k < POP_SIZE; k++)
          if (obj_val[k] < gen_best) gen_best = obj_val[k];
        Serial.print("Gen,");     Serial.print(cur_gen);
        Serial.print(",Best,");   Serial.print(best_obj_val,6);
        Serial.print(",GenBest,");Serial.print(gen_best,6);
        Serial.print(",RXtotal=");Serial.println(rx_total_msgs);
      }
      de_stage = (cur_gen >= MAX_GEN) ? DE_DONE : DE_NEXTGEN;
      break;
    }
    case DE_NEXTGEN: {
      delay(6);               // 延时6毫秒，或 delayMicroseconds(6000);
      de_stage = DE_EVOLVE;
      break;
    }
    default: {
      de_stage = DE_DONE;
      break;
    }
  }
}

//==================== 发送捕获片段 ====================//
void sendCaptureSegment(const float seg[SEG_LEN]) {
  delay(random(0,6));
  char p[MAXLEN];
  int  n = 0;
  for (uint8_t i = 0; i < SEG_LEN; i++) {
    dtostrf(seg[i],0,4,p+n);
    n += strlen(p+n);
    if (n >= MAXLEN-2) break;
    p[n++] = ',';
  }
  float r = randFloat01();
  dtostrf(r,0,4,p+n);
  n += strlen(p+n);
  if (n > MAXLEN-1) n = MAXLEN-1;
  p[n] = '\0';
  Wire.beginTransmission(IRCOMM_IR_ADDR);
  Wire.write((uint8_t*)p,n);
  Wire.endTransmission();
#if PRINT_TX
  Serial.print("TX ▶ "); Serial.println(p);
#endif
}

//==================== 串口命令处理 ====================//
void serialSourcePoll() {
  static char buf[16];
  static uint8_t len = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (len > 0) {
        buf[len] = '\0';
        if (!has_pending_cmd) {
          strncpy(pending_cmd, buf, sizeof(pending_cmd) - 1);
          pending_cmd[sizeof(pending_cmd) - 1] = '\0';
          has_pending_cmd = true;
        }
      }
      len = 0;
    } else if (len + 1 < sizeof(buf)) {
      buf[len++] = c;
    }
  }
}

void dispatchCommand() {
  if (!has_pending_cmd) return;
  if (equalsIgnoreCase(pending_cmd, "START")) {
    de_stage   = DE_INIT;
    cur_gen    = 0;
    de_enabled = true;
  }
  has_pending_cmd = false;
}
//==================== Arduino 主流程 ====================//
void setup() {
  Serial.begin(115200);
  Wire.begin(); 
  Wire.setClock(200000);
  randomSeed(quickSeedADC());
  ir_mode_t m = { MODE_START_RX };
  i2cWrite1(m.mode);
  for (uint8_t ch = 0; ch < 4; ch++) {
    m.mode = MODE_CLEAR_MSG0 + ch;
    i2cWrite1(m.mode);
  }
  Serial.println("Start (DE enabled; RX inject enabled)...");
}

void loop() {
  serialSourcePoll();
  dispatchCommand();
  task_rx_poll();
  if (de_enabled) {
    task_de_tick();
  }
}
