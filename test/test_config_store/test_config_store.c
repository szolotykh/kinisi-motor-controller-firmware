#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "config_store.h"
#include "hw_fram.h"

static uint8_t fram[8192], baseline[8192];
static int write_budget = -1, read_failure = -1, reads;
static bool write_protected;

bool hw_fram_read(uint16_t offset, void *data, uint16_t length)
{
    assert((size_t)offset + length <= sizeof(fram));
    if (reads++ == read_failure) return false;
    memcpy(data, fram + offset, length);
    return true;
}

bool hw_fram_write(uint16_t offset, const void *data, uint16_t length)
{
    assert((size_t)offset + length <= sizeof(fram));
    int count = length;
    if (write_budget >= 0) {
        if (count > write_budget) count = write_budget;
        write_budget -= count;
    }
    if (!write_protected) memcpy(fram + offset, data, (size_t)count);
    return count == length;
}

int main(void)
{
    const uint32_t first = 100, second = 201, third = 302;
    uint32_t restored = 999;
    memset(fram, 0xff, sizeof(fram));
    assert(!config_store_load(&restored, sizeof(restored)) && restored == 999);
    assert(config_store_save(&first, sizeof(first)));
    assert(config_store_save(&second, sizeof(second)));
    memcpy(baseline, fram, sizeof(fram));

    // A longer replacement must preserve the newest shorter legacy slot at
    // every interruption, including when that legacy slot is slot zero.
    const uint32_t extended[] = {401, 402, 403};
    for (int newest_slot = 1; newest_slot >= 0; --newest_slot) {
        uint8_t legacy_image[sizeof(fram)];
        memcpy(fram, baseline, sizeof(fram));
        if (newest_slot == 0) assert(config_store_save(&third, sizeof(third)));
        memcpy(legacy_image, fram, sizeof(fram));
        bool committed = false;
        for (int bytes = 0; bytes <= 255 && !committed; ++bytes) {
            memcpy(fram, legacy_image, sizeof(fram));
            write_budget = bytes;
            committed = config_store_save(extended, sizeof(extended));
            uint32_t actual[3] = {0};
            uint16_t size = config_store_load_latest(actual, sizeof(actual));
            if (committed) assert(size == sizeof(extended) && !memcmp(extended, actual, size));
            else assert(size == sizeof(first) && actual[0] == (newest_slot ? second : third));
        }
        assert(committed);
        write_budget = -1;
    }

    // Cut after every possible byte, including when replacing a previously
    // committed slot. Only the prior setup is selectable until commit.
    int cut = 0;
    for (; cut < 256; ++cut) {
        memcpy(fram, baseline, sizeof(fram));
        write_budget = cut;
        bool saved = config_store_save(&third, sizeof(third));
        assert(config_store_load(&restored, sizeof(restored)));
        assert(restored == (saved ? third : second));
        if (saved) break;
    }
    assert(cut > 1 && cut < 256);
    write_budget = -1;
    uint8_t other_length = 99;
    assert(!config_store_load(&other_length, sizeof(other_length)) && other_length == 99);
    // Damage the newest record and fall back to the remaining complete setup.
    fram[16] ^= 1;
    assert(config_store_load(&restored, sizeof(restored)) && restored == second);
    fram[256 + 16] ^= 1;
    assert(!config_store_load(&restored, sizeof(restored)));
    assert(config_store_save(&third, sizeof(third)));

    // Each failed read must suppress an ACK. Only a lost verification read
    // after the final marker was written may leave a complete newer record.
    memcpy(fram, baseline, sizeof(fram));
    reads = 0;
    assert(config_store_save(&third, sizeof(third)));
    int save_reads = reads;
    for (int i = 0; i < save_reads; ++i) {
        memcpy(fram, baseline, sizeof(fram));
        reads = 0;
        read_failure = i;
        assert(!config_store_save(&third, sizeof(third)));
        read_failure = -1;
        assert(config_store_load(&restored, sizeof(restored)));
        assert(restored == (i == save_reads - 1 ? third : second));
    }
    memcpy(fram, baseline, sizeof(fram));
    write_protected = true;
    assert(!config_store_save(&third, sizeof(third)));
    assert(config_store_load(&restored, sizeof(restored)) && restored == second);
    printf("PASS journal: %d interrupted writes, %d read failures, corruption and write protection\n",
           cut, save_reads);
}
