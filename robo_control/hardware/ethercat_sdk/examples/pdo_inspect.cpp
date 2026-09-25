// Dumps each slave's real PDO mapping via SDO so the hardcoded RxPDO/TxPDO
// structs in types.hpp can be checked against the drive's actual layout.
#include <soem/soem.h>

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>

static ecx_contextt ctx;

static uint8_t readU8(int slave, uint16_t idx, uint8_t sub) {
  uint8_t v = 0;
  int size = sizeof(v);
  int wkc = ecx_SDOread(&ctx, slave, idx, sub, FALSE, &size, &v, EC_TIMEOUTRXM);
  if (wkc <= 0)
    printf("    [SDO read 0x%04X:%02X failed, wkc=%d]\n", idx, sub, wkc);
  return v;
}

static uint16_t readU16(int slave, uint16_t idx, uint8_t sub) {
  uint16_t v = 0;
  int size = sizeof(v);
  int wkc = ecx_SDOread(&ctx, slave, idx, sub, FALSE, &size, &v, EC_TIMEOUTRXM);
  if (wkc <= 0)
    printf("    [SDO read 0x%04X:%02X failed, wkc=%d]\n", idx, sub, wkc);
  return v;
}

static uint32_t readU32(int slave, uint16_t idx, uint8_t sub) {
  uint32_t v = 0;
  int size = sizeof(v);
  int wkc = ecx_SDOread(&ctx, slave, idx, sub, FALSE, &size, &v, EC_TIMEOUTRXM);
  if (wkc <= 0)
    printf("    [SDO read 0x%04X:%02X failed, wkc=%d]\n", idx, sub, wkc);
  return v;
}

// Name the objects we care about for this drive.
static const char *objName(uint16_t idx) {
  switch (idx) {
  case 0x6040: return "control_word";
  case 0x6041: return "status_word";
  case 0x6060: return "mode_of_operation";
  case 0x6061: return "mode_display";
  case 0x6064: return "position_actual";
  case 0x606C: return "velocity_actual";
  case 0x6071: return "target_torque";
  case 0x6072: return "max_torque";
  case 0x6077: return "torque_actual";
  case 0x607A: return "target_position";
  case 0x60B0: return "position_offset";
  case 0x60B1: return "velocity_offset";
  case 0x60B2: return "torque_offset";
  case 0x60FF: return "target_velocity";
  case 0x603F: return "error_code";
  case 0x0000: return "<padding>";
  default:     return "?";
  }
}

static void dumpPdoMap(int slave, uint16_t pdo_idx) {
  uint8_t n = readU8(slave, pdo_idx, 0x00);
  printf("  PDO 0x%04X: %u entries\n", pdo_idx, n);
  int bit_off = 0;
  for (uint8_t i = 1; i <= n; i++) {
    uint32_t e = readU32(slave, pdo_idx, i);
    uint16_t idx = static_cast<uint16_t>(e >> 16);
    uint8_t sub = static_cast<uint8_t>((e >> 8) & 0xFF);
    uint8_t bits = static_cast<uint8_t>(e & 0xFF);
    printf("    byte %2d : 0x%04X:%02X  %2u bits  %s\n", bit_off / 8, idx, sub,
           bits, objName(idx));
    bit_off += bits;
  }
  printf("    total: %d bytes\n", bit_off / 8);
}

static void dumpAssign(int slave, uint16_t sm_idx, const char *label) {
  uint8_t n = readU8(slave, sm_idx, 0x00);
  printf("  %s assign 0x%04X: %u PDO(s) ->", label, sm_idx, n);
  for (uint8_t i = 1; i <= n; i++)
    printf(" 0x%04X", readU16(slave, sm_idx, i));
  printf("\n");
}

int main(int argc, char *argv[]) {
  if (argc < 2) {
    fprintf(stderr, "Usage: sudo %s <interface>\n", argv[0]);
    return 1;
  }

  if (ecx_init(&ctx, argv[1]) <= 0) {
    fprintf(stderr, "Failed to init interface %s\n", argv[1]);
    return 1;
  }
  if (ecx_config_init(&ctx) <= 0) {
    fprintf(stderr, "No slaves found on %s\n", argv[1]);
    ecx_close(&ctx);
    return 1;
  }

  printf("Found %d slave(s) on %s\n\n", ctx.slavecount, argv[1]);

  // SDO mailbox needs the slave confirmed in PRE_OP first (mirrors the SDK).
  for (int s = 1; s <= ctx.slavecount; s++) {
    ctx.slavelist[s].state = EC_STATE_PRE_OP;
    ecx_writestate(&ctx, s);
    ecx_statecheck(&ctx, s, EC_STATE_PRE_OP, EC_TIMEOUTSTATE);
  }
  ecx_readstate(&ctx);

  for (int s = 1; s <= ctx.slavecount; s++) {
    printf("slave %d state = 0x%02X\n", s, ctx.slavelist[s].state);
  }
  printf("\n");

  for (int s = 1; s <= ctx.slavecount; s++) {
    printf("=== slave %d (%s) ===\n", s, ctx.slavelist[s].name);
    dumpAssign(s, 0x1C12, "RxPDO");
    dumpAssign(s, 0x1C13, "TxPDO");
    // Dump the candidate maps directly, whether assigned or not.
    for (uint16_t pdo : {0x1600, 0x1601}) dumpPdoMap(s, pdo);
    for (uint16_t pdo : {0x1A00, 0x1A01}) dumpPdoMap(s, pdo);
    printf("\n");
  }

  ecx_close(&ctx);
  return 0;
}
