#include <Arduino.h>
#include <Wire.h>
#include "ircomm_i2c.h"

//==================== 宏定义 ====================//
#define PRINT_TX           1    // 打印发送负载
#define PRINT_RX           1    // 打印接收负载
#define PRINT_GA_LOG_EVERY 0    // 每多少代打印一次GA日志；0=每代
#define ELITE_COUNT        2    // 每代保留的精英个体数
#define IMM_PER_GEN        1    // “移民”注入参数
#define MAXLEN             32   // I²C 单事务最大
#define RX_DRAIN_MAX       16   // 单次最大读出条数

//==================== 通用参数 ====================//
static const uint8_t  DIM             = 7;
static const uint8_t  POP_SIZE        = 10;
static const uint16_t MAX_GEN         = 5000;
static const float    X_MIN           = -5.12f;
static const float    X_MAX           =  5.12f;
static const uint16_t CAPTURE_INTERVAL= 30;   // 捕获间隔代数
static const uint8_t  SEG_LEN         = 3;
static const uint16_t RX_POLL_MS      = 2;    // 接收轮询间隔

//==================== GA 参数 ====================//
static const float CROSSOVER_RATE = 0.8f;
static float       MUTATION_RATE  = 0.14f;
static float       MUTATION_RANGE = 1.0f;

//==================== 全局数据区 ====================//
// GA 种群与统计
float    population[POP_SIZE][DIM];
float    new_population[POP_SIZE][DIM];
float    obj_val[POP_SIZE];
float    fitness_val[POP_SIZE];
uint8_t  mating_pool[POP_SIZE];
float    best_solution[DIM];
float    best_obj_val;
uint16_t cur_gen = 0;

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

// 控制开关 & 命令
bool    ga_enabled      = false;
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

float randFloat01()               { return random(0,10000) / 10000.0f; }
float randFloat(float lo,float hi){ return lo + (hi - lo) * randFloat01(); }

float rastrigin(const float *x) {
  float s = 10.0f * DIM;
  for (uint8_t i = 0; i < DIM; i++) {
    s += x[i] * x[i] - 10.0f * cos(2.0f * PI * x[i]);
  }
  return s;
}

float calcFitness(float f) { return 1.0f / (1.0f + f); }

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
      Serial.print(rxPool[idx].segment[j], 4);
      if (j < SEG_LEN - 1) Serial.print(",");
    }
    Serial.println();
#endif

    for (uint8_t j = 0; j < SEG_LEN; j++) {
      population[victim][start + j] = rxPool[idx].segment[j];
    }
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
    Wire.requestFrom(IRCOMM_IR_ADDR, 1);
    if (Wire.available() < 1) return false;
    uint8_t len = Wire.read();
    if (len == 0 || len >= MAXLEN) continue;
    m.mode = MODE_REPORT_MSG0 + ch;
    if (!i2cWrite1(m.mode)) return false;
    Wire.requestFrom(IRCOMM_IR_ADDR, len);
    if (Wire.available() < len) return false;
    for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
    buf[len] = '\0';
    outLen = len;
    if (buf[0] == '!') continue;
    return true;
  }
  return false;
}

static bool recvDataChunk(float *dataOut, uint8_t maxLen, uint8_t &outCount) {
  char buf[MAXLEN];
  uint8_t len;
  if (!recvRaw(buf, len) || !strchr(buf, ',')) return false;
  rx_total_msgs++;
  outCount = 0;
  char *p = buf;
  while (outCount < maxLen && p) {
    dataOut[outCount++] = atof(p);
    p = strchr(p, ',');
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
    float data[SEG_LEN+1];
    uint8_t cnt;
    if (!(recvDataChunk(data, SEG_LEN+1, cnt) && cnt == SEG_LEN+1)) break;
    storeRxSegment(data, data[SEG_LEN]);
    drained++;
  }
}

//==================== GA 相关：选择 & 状态机 ====================//
static float cumF[POP_SIZE];
void selectionRoulette() {
  float totalF = 0;
  for (uint8_t i = 0; i < POP_SIZE; i++) totalF += fitness_val[i];
  cumF[0] = fitness_val[0];
  for (uint8_t i = 1; i < POP_SIZE; i++) cumF[i] = cumF[i-1] + fitness_val[i];
  for (uint8_t k = 0; k < POP_SIZE; k++) {
    float r = randFloat(0.0f, totalF);
    uint8_t idx = 0;
    while (idx < POP_SIZE && cumF[idx] < r) idx++;
    mating_pool[k] = min(idx, POP_SIZE-1);
  }
}

enum GAStage {
  GA_INIT, GA_EVAL, GA_SELECT, GA_CROSS_MUT,
  GA_REPLACE, GA_EVAL2, GA_UPDATE_BEST,
  GA_CAPTURE_SEND, GA_LOG, GA_NEXTGEN, GA_DONE
};
GAStage ga_stage = GA_INIT;
uint8_t   ga_i    = 0;

void ga_init_population() {
  for (uint8_t i = 0; i < POP_SIZE; i++)
    for (uint8_t j = 0; j < DIM; j++)
      population[i][j] = randFloat(X_MIN, X_MAX);
}

void task_ga_tick() {
  if (ga_stage == GA_DONE) return;
  switch (ga_stage) {
    case GA_INIT: {
      uint32_t seed = quickSeedADC();
      randomSeed(seed);
      Serial.print("Seed,0x"); Serial.print(seed, HEX);
      Serial.print(","); Serial.println(seed);
      ga_init_population();
      cur_gen = 0;
      rx_total_msgs = 0;
      ga_stage = GA_EVAL;
      ga_i = 0;
      break;
    }
    case GA_EVAL: {
      obj_val[ga_i]     = rastrigin(population[ga_i]);
      fitness_val[ga_i] = calcFitness(obj_val[ga_i]);
      if (++ga_i >= POP_SIZE) {
        best_obj_val = obj_val[0];
        memcpy(best_solution, population[0], DIM * sizeof(float));
        for (uint8_t k = 1; k < POP_SIZE; k++) {
          if (obj_val[k] < best_obj_val) {
            best_obj_val = obj_val[k];
            memcpy(best_solution, population[k], DIM * sizeof(float));
          }
        }
        ga_stage = GA_SELECT;
        ga_i = 0;
      }
      break;
    }
    case GA_SELECT: {
      selectionRoulette();
      ga_stage = GA_CROSS_MUT;
      ga_i = 0;
      break;
    }
    case GA_CROSS_MUT: {
      if (ga_i >= POP_SIZE) {
        ga_stage = GA_REPLACE;
        ga_i = 0;
        break;
      }
      uint8_t p1 = mating_pool[ga_i];
      uint8_t p2 = mating_pool[ga_i+1];
      if (randFloat01() < CROSSOVER_RATE) {
        float a = randFloat01();
        for (uint8_t j = 0; j < DIM; j++) {
          new_population[ga_i][j]   = a * population[p1][j] + (1 - a) * population[p2][j];
          new_population[ga_i+1][j] = (1 - a) * population[p1][j] + a * population[p2][j];
        }
      } else {
        memcpy(new_population[ga_i],   population[p1],   DIM * sizeof(float));
        memcpy(new_population[ga_i+1], population[p2], DIM * sizeof(float));
      }
      for (uint8_t j = 0; j < DIM; j++) {
        if (randFloat01() < MUTATION_RATE) {
          float d = randFloat(-MUTATION_RANGE, MUTATION_RANGE);
          new_population[ga_i][j] = constrain(new_population[ga_i][j] + d, X_MIN, X_MAX);
        }
        if (randFloat01() < MUTATION_RATE) {
          float d = randFloat(-MUTATION_RANGE, MUTATION_RANGE);
          new_population[ga_i+1][j] = constrain(new_population[ga_i+1][j] + d, X_MIN, X_MAX);
        }
      }
      ga_i += 2;
      break;
    }
    case GA_REPLACE: {
      if (ga_i < ELITE_COUNT) {
        memcpy(population[ga_i], best_solution, DIM * sizeof(float));
        obj_val[ga_i]     = best_obj_val;
        fitness_val[ga_i] = calcFitness(best_obj_val);
      } else {
        memcpy(population[ga_i], new_population[ga_i], DIM * sizeof(float));
      }
      if (++ga_i >= POP_SIZE) {
        ga_stage = GA_EVAL2;
        ga_i = 0;
      }
      break;
    }
    case GA_EVAL2: {
      obj_val[ga_i]     = rastrigin(population[ga_i]);
      fitness_val[ga_i] = calcFitness(obj_val[ga_i]);
      if (++ga_i >= POP_SIZE) {
        ga_stage = GA_UPDATE_BEST;
        ga_i = 0;
      }
      break;
    }
    case GA_UPDATE_BEST: {
      for (uint8_t k = 0; k < POP_SIZE; k++) {
        if (obj_val[k] < best_obj_val) {
          best_obj_val = obj_val[k];
          memcpy(best_solution, population[k], DIM * sizeof(float));
        }
      }
      ga_stage = GA_CAPTURE_SEND;
      break;
    }
    case GA_CAPTURE_SEND: {
      cur_gen++;
      if (cur_gen % CAPTURE_INTERVAL == 0) {
        uint8_t idx = 0;
        float gen_best = obj_val[0];
        for (uint8_t i = 1; i < POP_SIZE; i++) {
          if (obj_val[i] < gen_best) {
            gen_best = obj_val[i];
            idx = i;
          }
        }
        float seg[SEG_LEN];
        uint8_t s = random(0, DIM - SEG_LEN + 1);
        for (uint8_t j = 0; j < SEG_LEN; j++) {
          seg[j] = population[idx][s + j];
        }
        sendCaptureSegment(seg);
        injectRxSegmentsIntoPopulation();
      }
      ga_stage = GA_LOG;
      break;
    }
    case GA_LOG: {
      float gen_best = obj_val[0];
      for (uint8_t k = 1; k < POP_SIZE; k++) {
        if (obj_val[k] < gen_best) gen_best = obj_val[k];
      }
      if (PRINT_GA_LOG_EVERY == 0 || cur_gen % PRINT_GA_LOG_EVERY == 0) {
        Serial.print("Gen,");     Serial.print(cur_gen);
        Serial.print(",Best,");   Serial.print(best_obj_val, 6);
        Serial.print(",GenBest,");Serial.print(gen_best, 6);
        Serial.print(",RXtotal=");Serial.println(rx_total_msgs);
      }
      ga_stage = (cur_gen >= MAX_GEN) ? GA_DONE : GA_NEXTGEN;
      break;
    }
    case GA_NEXTGEN: {
      ga_stage = GA_SELECT;
      ga_i = 0;
      break;
    }
    default: {
      ga_stage = GA_DONE;
      break;
    }
  }
}

//==================== 发送捕获片段 ====================//
void sendCaptureSegment(const float seg[SEG_LEN]) {
  delay(random(0, 6));
  char p[MAXLEN];
  int n = 0;
  for (uint8_t i = 0; i < SEG_LEN; i++) {
    dtostrf(seg[i], 0, 4, p + n);
    n += strlen(p + n);
    if (n >= MAXLEN - 2) break;
    p[n++] = ',';
  }
  float r = randFloat01();
  dtostrf(r, 0, 4, p + n);
  n += strlen(p + n);
  if (n > MAXLEN - 1) n = MAXLEN - 1;
  p[n] = '\0';
  Wire.beginTransmission(IRCOMM_IR_ADDR);
  Wire.write((uint8_t *)p, n);
  Wire.endTransmission();
#if PRINT_TX
  Serial.print("TX ▶ ");
  Serial.println(p);
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
    ga_stage   = GA_INIT;
    cur_gen    = 0;
    ga_enabled = true;
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
  Serial.println("Start (同步插入 & 调试追踪)...");
}

void loop() {
  serialSourcePoll();
  dispatchCommand();
  task_rx_poll();
  if (ga_enabled) {
    task_ga_tick();
  }
}
