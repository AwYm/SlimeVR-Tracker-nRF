#!/usr/bin/env python3
"""Run production battery sampling with a host model of Zephyr's ADC API.

The model is deliberately local: host tests must not depend on a developer's NCS
checkout.  Its gain ratios and integer conversion order mirror Zephyr's public
``adc_raw_to_microvolts()`` contract.
"""
import os
from pathlib import Path
import re
import shlex
import subprocess
import tempfile

HERE = Path(__file__).resolve().parent
ROOT = Path(os.environ.get("SOURCE_ROOT", HERE.parents[2]))
battery = (ROOT / "src/system/battery.c").read_text()


def function(source, name):
    match = re.search(rf"^(?:static inline )?int {name}\([^;{{]*\)\s*\{{", source, re.MULTILINE)
    if match is None:
        raise ValueError(name)
    start = source.index("{", match.start())
    tokens = re.compile(r'/\*.*?\*/|//[^\n]*|"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'|[{}]', re.DOTALL)
    depth = 0
    for token in tokens.finditer(source, start):
        if token.group() == "{":
            depth += 1
        elif token.group() == "}":
            depth -= 1
            if depth == 0:
                return source[match.start():token.end()]
    raise ValueError(f"Unclosed function: {name}")


prefix = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <errno.h>
#include <limits.h>
#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#define __ASSERT_NO_MSG(x) assert(x)
#define __ASSERT_MSG_INFO(...) assert(0)
#define LOG_INF(...) ((void)0)
#define USE_PMIC_CHARGER 0

enum adc_gain {
    ADC_GAIN_1_6,
    ADC_GAIN_1_5,
    ADC_GAIN_1_4,
    ADC_GAIN_1_3,
    ADC_GAIN_1_2,
    ADC_GAIN_2_3,
    ADC_GAIN_1,
    ADC_GAIN_2,
    ADC_GAIN_3,
    ADC_GAIN_4,
    ADC_GAIN_6,
    ADC_GAIN_8,
    ADC_GAIN_12,
    ADC_GAIN_16,
    ADC_GAIN_24,
    ADC_GAIN_32,
    ADC_GAIN_64,
    ADC_GAIN_128,
};

static int adc_gain_ratio(enum adc_gain gain, int32_t *numerator, int32_t *denominator) {
    static const struct { int16_t numerator; int16_t denominator; } ratios[] = {
        {1, 6}, {1, 5}, {1, 4}, {1, 3}, {1, 2}, {2, 3}, {1, 1},
        {2, 1}, {3, 1}, {4, 1}, {6, 1}, {8, 1}, {12, 1}, {16, 1},
        {24, 1}, {32, 1}, {64, 1}, {128, 1},
    };
    if ((unsigned int)gain >= ARRAY_SIZE(ratios)) {
        return -EINVAL;
    }
    *numerator = ratios[gain].numerator;
    *denominator = ratios[gain].denominator;
    return 0;
}

static int adc_raw_to_microvolts(int32_t ref_mv, enum adc_gain gain,
                                 uint8_t resolution, int32_t *value) {
    int32_t numerator;
    int32_t denominator;
    if (resolution == 0 || resolution > 31 ||
        adc_gain_ratio(gain, &numerator, &denominator) != 0) {
        return -EINVAL;
    }
    int64_t microvolts = (int64_t)*value * ref_mv * 1000 * denominator;
    microvolts /= numerator;
    microvolts /= (INT64_C(1) << resolution);
    if (microvolts < INT32_MIN || microvolts > INT32_MAX) {
        return -ERANGE;
    }
    *value = (int32_t)microvolts;
    return 0;
}
'''
parts = [prefix]
parts += [r'''
struct device { int unused; };
struct adc_sequence { uint8_t resolution; bool calibrate; };
struct divider_data {
    const struct device *adc;
    struct { enum adc_gain gain; } adc_cfg;
    struct adc_sequence adc_seq;
    int16_t raw;
};
struct divider_config { uint32_t output_ohm, full_ohm; };
static struct divider_data divider_data;
static struct divider_config divider_config;
static bool battery_ok;
static int read_result;
static int adc_read(const struct device *dev, struct adc_sequence *seq) {
    (void)dev; (void)seq;
    return read_result;
}
static uint16_t adc_ref_internal(const struct device *dev) {
    (void)dev;
    return 600;
}
''', function(battery, "battery_sample"), r'''
static int sample(int16_t raw, enum adc_gain gain, uint32_t output, uint32_t full) {
    divider_data.raw = raw;
    divider_data.adc_cfg.gain = gain;
    divider_data.adc_seq.resolution = 14;
    divider_config.output_ohm = output;
    divider_config.full_ohm = full;
    return battery_sample();
}
int main(void) {
    battery_ok = true;
    /* P10: VDDHDIV5, 600 mV reference, gain 1/2, 14-bit ADC.
     * 10004 codes represent 3.66357421875 V before integer quantization. */
    assert(sample(10000, ADC_GAIN_1_2, 1, 5) == 3662);
    assert(sample(10004, ADC_GAIN_1_2, 1, 5) == 3663);
    assert(sample(11469, ADC_GAIN_1_2, 1, 5) == 4200);
    assert(sample(0, ADC_GAIN_1_2, 1, 5) == 0);
    assert(sample(-10000, ADC_GAIN_1_2, 1, 5) == -3662);
    /* Direct VDD: the public result remains integer millivolts. */
    assert(sample(15023, ADC_GAIN_1_6, 0, 0) == 3300);
    /* External divider, including a product exceeding signed 32-bit range. */
    assert(sample(5000, ADC_GAIN_1_6, 100000, 330000) == 3625);
    assert(sample(5000, ADC_GAIN_1_6, 1000000000, 3300000000U) == 3625);
    /* Zephyr rejects invalid gain; it must never become a voltage reading. */
    assert(sample(10004, (enum adc_gain)255, 1, 5) == -EINVAL);
    read_result = -EIO;
    assert(sample(10004, ADC_GAIN_1_2, 1, 5) == -EIO);
    read_result = 0;
    battery_ok = false;
    assert(sample(10004, ADC_GAIN_1_2, 1, 5) == -ENOENT);
    puts("PASS production battery precision: P10, direct VDD, external divider, signed scaling and errors");
    return 0;
}
''']
with tempfile.TemporaryDirectory(prefix="battery-precision-") as tmp:
    source = Path(tmp) / "test.c"
    executable = Path(tmp) / "test"
    source.write_text("\n".join(parts))
    subprocess.run(shlex.split(os.environ.get("CC", "cc")) + [
        "-std=c11", "-Wall", "-Wextra", "-Werror", "-fsanitize=undefined",
        "-fno-sanitize-recover=all", str(source), "-o", str(executable)], check=True)
    subprocess.run([str(executable)], check=True)
