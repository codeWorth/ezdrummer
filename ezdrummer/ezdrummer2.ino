#include <ctype.h>
#include <Arduino_LSM6DS3.h>

// Motor connections
const int speed = 9;
const int in1 = 16;
const int in2 = 17;
const int switchPin = 8;

struct Hits;
struct HitPayload;

const int GRID_SIZE = 32;
int grid[GRID_SIZE];
int player_grid[GRID_SIZE];

// Hit contains time and strength, totaling 2 bytes
// HIT_TIME_POWER of the bits are time, the remaining bits are strength
const int HIT_TIME_POWER = 30;
const uint32_t HIT_TIME_MASK = 0xffffffff >> (32 - HIT_TIME_POWER);
const uint32_t HIT_STRENGTH_POWER = 32 - HIT_TIME_POWER;
const uint32_t HIT_STRENGTH_MAX = 1 << HIT_STRENGTH_POWER;
const uint32_t HIT_STRENGTH_MASK = 0xffffffff >> HIT_TIME_POWER;
struct Hit {
  uint32_t data;
  // data[0:30] = time in milliseconds
  // data[30:32] = strength
};

uint32_t hit_t(const Hit& hit) {
  return hit.data & HIT_TIME_MASK;
}
uint8_t hit_strength(const Hit& hit) {
  return hit.data >> HIT_TIME_POWER;
}
void set_hit_values(Hit& hit, uint32_t time, uint8_t strength) {
  hit.data = time & HIT_TIME_MASK;
  hit.data |= strength << HIT_TIME_POWER;
}


// Circular buffer of max size 2^HIT_LIST_CAPACITY_POWER
const int HIT_LIST_CAPACITY_POWER = 9;
const uint16_t HIT_LIST_CAPACITY = 1 << HIT_LIST_CAPACITY_POWER;
const uint16_t HIT_LIST_INDEX_MASK = 0xFFFF >> (16 - HIT_LIST_CAPACITY_POWER);
struct Hits {
  Hit hits[HIT_LIST_CAPACITY];
  uint16_t start;   // inclusive bound
  uint16_t end;     // exclusive bound
};

uint16_t hits_count(const Hits& hits) {
  uint16_t diff = hits.end - hits.start;
  if (diff <= 0) {
    return HIT_LIST_CAPACITY - diff;
  } else {
    return diff;
  }
}

const Hit& hit_at(const Hits& hits, uint16_t index) {
  index += hits.start;
  index &= HIT_LIST_INDEX_MASK;
  return hits.hits[index];
}

void push_hit(Hits& hits, uint32_t time, uint8_t strength) {
  set_hit_values(hits.hits[hits.end], time, strength);

  if (hits.end + 1 == hits.start) { // if we're about to be at max capacity
    hits.start++;                   // get rid of the earliest added element
    hits.start &= HIT_LIST_INDEX_MASK;
  }
  hits.end++;
  hits.end &= HIT_LIST_INDEX_MASK;  // loop around to the start if we exceeded max index
}

const Hit& pop_hit(Hits& hits) {
  if (hits.end == 0) {
    hits.end = HIT_LIST_CAPACITY - 1;
  } else {
    hits.end--;
  }

  return hits.hits[hits.end];
}

void clear_hits(Hits& hits) {
  hits.start = 0;
  hits.end = 0;
}


const int RAISE_STICK_TIME = 60;
const int RELEASE_STICK_TIME = 140;
int hitTime = -1;
int releaseTime = -1;

int getHitPower(int power, int maxPower) {
    return (RAISE_STICK_TIME * (power + 1)) / maxPower;
}

int getReleasePower(int power, int maxPower) {
  return (RELEASE_STICK_TIME * (power + 1)) / maxPower;
}

void hit(int power, int maxPower) {
  const int hitPower = getHitPower(power, maxPower);
  const int releasePower = getReleasePower(power, maxPower);

  // raise stick up
  analogWrite(speed, 200);
  digitalWrite(in1, HIGH);
	digitalWrite(in2, LOW);
  hitTime = millis() + hitPower;
  releaseTime = hitTime + releasePower;
}

void doHit() {
  if (hitTime != -1 && millis() > hitTime) {
    // do hit
    analogWrite(speed, 200);
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    hitTime = -1;
  }
  if (releaseTime != -1 && millis() > releaseTime) {
    // release power
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    releaseTime = -1;
  }
}

void write_pattern(int* grid, const Hits& hits, uint32_t pattern_length) {
  const uint32_t smallest = pattern_length / GRID_SIZE;
  const uint16_t hits_length = hits_count(hits);

  for (int n = 0; n < GRID_SIZE; n++) {
    grid[n] = 0;
  }
  
  for (int i = 0; i < hits_length; i++) {
    const Hit hit = hit_at(hits, i);
    const int t_hit = hit_t(hit) % pattern_length;
    int dt_min = 0;
    int n_min = -1;

    for (int n = 0; n < GRID_SIZE; n++) {
      int t = smallest * n;
      int dt = abs(t_hit - t);
      if (n_min == -1 || dt < dt_min) {
        dt_min = dt;
        n_min = n;
      }
    }

    grid[n_min] += hit_strength(hit) + 1;
  }
}

const float HIT_THRESH = 1.8f;
const float HIT_RETURN = 1.5f;
const float MAX_A = 7.0f;
const int RETURN_THRESH = 20;
const int DONE_MILLIS = 5000;
uint8_t accelToHitStrength(float accel) {
  accel = (accel - HIT_THRESH) / (MAX_A - HIT_THRESH) * HIT_STRENGTH_MAX;
  int accel_int = min(accel, HIT_STRENGTH_MAX - 1);;
  return accel_int & HIT_STRENGTH_MASK;
}

Hits hits;
int t0;
int return_count;
int hit_time;
float max_a;
void resetHitGlobals() {
  t0 = -1;
  return_count = RETURN_THRESH;
  hit_time = -1;
  max_a = -1;
}

void logHit(int time, float acceleration, Hits& hits) {
  int strength = accelToHitStrength(acceleration);
  Serial.print(time);
  Serial.print(", ");
  Serial.println(strength);
  push_hit(hits, time, strength);
}

bool hitsDone() {
  return hit_time != -1 && (millis() - hit_time) > DONE_MILLIS;
}

struct HitPayload {
  int time;
  float accel;
};

// Check if a hit has happened
// Returns a payload with the hit if so
// Returns a payload with time == -1 if not
HitPayload checkHit() {
  HitPayload payload;

  if (IMU.accelerationAvailable()) {
    float ax, ay, az;
    IMU.readAcceleration(ax, ay, az);
    float a = sqrt(ax*ax + ay*ay + az*az);

    if (t0 == -1) { // if waiting for initial hit
      if (a > HIT_THRESH) { // if we got the hit, save it
        max_a = a;
        hit_time = millis();
        t0 = hit_time;      // update t0 to first hit time
        return_count = 0;   // count for hit to end
      }
    } else if (a < HIT_RETURN) {  // if the hit is ending
      return_count++;             // increase count of below thresh
      if (return_count == RETURN_THRESH) {  // if we reached enough count
        payload.time = hit_time - t0;
        payload.accel = max_a;
        max_a = 0;
        return payload;
      }
    } else if (a > HIT_THRESH) {  // if the hit is happening
      if (return_count >= RETURN_THRESH) {  // if we have waited long enough
        hit_time = millis();                // consider this a new hit
      }
      max_a = max(a, max_a);                // update maximum accel
      return_count = 0;                     // clear times below thresh
    }
  }

  payload.time = -1;
  return payload;
}

bool playMode() {
  return digitalRead(switchPin) == 1;
}

void printGrid(int* grid) {
  for (int i = 0; i < GRID_SIZE; i++) {
    Serial.print(grid[i]);
    Serial.print(' ');
    Serial.print(' ');
  }
  Serial.print('\n');
}

int playerLoopLength;
int playerHitCount;
int hitsInPattern;
int playerT0;
int lastDt;
bool ignoreNextHit;

void resetPlayerData() {
  playerLoopLength = -1;
  playerHitCount = 0;
  hitsInPattern = 0;
  playerT0 = 0;
  lastDt = -1;
  ignoreNextHit = false;
}

bool read_mode = true;
bool read_part_mode = false;

void setup() {
  Serial.begin(9600);
  // while (!Serial);
  
  // initialize IMU
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1)
      ;
  }

  Serial.println("Initialized IMU"); 

  pinMode(speed, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(switchPin, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);

  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  analogWrite(speed, 100);
  digitalWrite(LED_BUILTIN, HIGH);

  resetHitGlobals();
  resetPlayerData();
}

int nextHit(int* grid, int t, int loopLength) {
  for (int i = GRID_SIZE - 1; i >= 0; i--) {
    if (grid[i] == 0) continue;
    const int hitT = (i * loopLength) / GRID_SIZE - getHitPower(grid[i] - 1, HIT_STRENGTH_MAX) + getReleasePower(grid[i] - 1, HIT_STRENGTH_MAX) - 20;
    if (t > hitT) {
      return i;
    }
  }
  return -1;
}

void loop() {
  if (!playMode()) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    if (playerHitCount > 0) { // if we aren't in play mode, but we were before
      read_mode = true;       // go back into read mode
    }
  }

  if (read_mode) {
    if (playMode()) return;

    digitalWrite(LED_BUILTIN, HIGH);
    bool finished = hitsDone();
    if (finished) {
      read_mode = false;

      Hit last_hit = pop_hit(hits);
      write_pattern(grid, hits, hit_t(last_hit));
      printGrid(grid);

      clear_hits(hits);
      read_part_mode = true;
      resetHitGlobals();
    } else {
      HitPayload payload = checkHit();
      if (payload.time != -1) {
        logHit(payload.time, payload.accel, hits);
      }
    }
  } else if (read_part_mode) {
    if (playMode()) return;

    digitalWrite(LED_BUILTIN, LOW);
    bool finished = hitsDone();
    if (finished) {
      read_part_mode = false;

      Hit last_hit = pop_hit(hits);
      write_pattern(player_grid, hits, hit_t(last_hit));
      printGrid(player_grid);

      resetHitGlobals();
      resetPlayerData();
      hitsInPattern = hits_count(hits);
      Serial.print(hitsInPattern);
      Serial.println(" hits in player pattern");
      clear_hits(hits);
    } else {
      HitPayload payload = checkHit();
      if (payload.time != -1) {
        logHit(payload.time, payload.accel, hits);
      }
    }
  } else if (playMode()) {    
    HitPayload payload = checkHit();
    if (playerLoopLength == -1) {
      if (payload.time != -1) {
        playerHitCount++;
        Serial.print("Listening.. ");
        Serial.println(playerHitCount);
      }

      if (playerHitCount == hitsInPattern + 1) {
        Serial.print("Loop len = ");
        Serial.println(payload.time - playerT0);
        playerLoopLength = payload.time - playerT0;
        playerT0 = payload.time;
      }
    } else {
      doHit(); // triggers actuator if needed

      const int dt = (millis() - playerT0 - t0) % playerLoopLength;
      const int nextHitIndex = nextHit(grid, dt, playerLoopLength);
      int lastHitIndex = -1;
      if (lastDt == -1) {
        lastDt = dt;
      } else {
        lastHitIndex = nextHit(grid, lastDt, playerLoopLength);
        lastDt = dt;
      }

      if (nextHitIndex != lastHitIndex) {
        const int playerPower = player_grid[nextHitIndex];
        const int power = grid[nextHitIndex];
        if (playerPower == 0 && power > 0) {
          Serial.println("HITTING!");
          hit(power - 1, HIT_STRENGTH_MAX);
          ignoreNextHit = true;
        }
      }

      if (payload.time != -1) {
        if (ignoreNextHit) {
          ignoreNextHit = false;
        } else {
          playerHitCount++;
          Serial.print("Playing.. ");
          Serial.println(playerHitCount);
        }
      }
    }

    // if (playerHitCount == hitsInPattern + 1) {
    //   Serial.print("Loop len = ");
    //   Serial.println(payload.time - playerT0);
    //   if (playerLoopLength == -1) {
    //     playerLoopLength = payload.time - playerT0;
    //   } else {
    //     int loopTime = (payload.time - playerT0) % playerLoopLength;
    //     if (loopTime > playerLoopLength / 2) {
    //       loopTime -= playerLoopLength;
    //     }
    //     playerLoopLength += loopTime;
    //   }
    //   playerHitCount = 0;
    //   playerT0 = payload.time;
    // }
  } else {
    digitalWrite(LED_BUILTIN, HIGH);
  }
}
