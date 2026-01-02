// protopirate_history.c
#include "protopirate_history.h"
#include <lib/subghz/receiver.h>
#include <flipper_format/flipper_format_i.h>

#define TAG "ProtoPirateHistory"

typedef struct {
    FuriString* item_str;
    FlipperFormat* flipper_format;
    uint8_t type;
    SubGhzRadioPreset* preset;
} ProtoPirateHistoryItem;

ARRAY_DEF(ProtoPirateHistoryItemArray, ProtoPirateHistoryItem, M_POD_OPLIST)

struct ProtoPirateHistory {
    ProtoPirateHistoryItemArray_t data;
    uint16_t last_index;
    uint32_t last_update_timestamp;
    uint8_t code_last_hash_data;
};

ProtoPirateHistory* protopirate_history_alloc(void) {
    ProtoPirateHistory* instance = malloc(sizeof(ProtoPirateHistory));
    ProtoPirateHistoryItemArray_init(instance->data);
    instance->last_index = 0;
    return instance;
}

void protopirate_history_free(ProtoPirateHistory* instance) {
    furi_assert(instance);
    for(size_t i = 0; i < ProtoPirateHistoryItemArray_size(instance->data); i++) {
        ProtoPirateHistoryItem* item = ProtoPirateHistoryItemArray_get(instance->data, i);
        furi_string_free(item->item_str);
        flipper_format_free(item->flipper_format);
        if(item->preset) {
            if(item->preset->name) {
                furi_string_free(item->preset->name);
            }
            free(item->preset);
        }
    }
    ProtoPirateHistoryItemArray_clear(instance->data);
    free(instance);
}

void protopirate_history_reset(ProtoPirateHistory* instance) {
    furi_assert(instance);
    for(size_t i = 0; i < ProtoPirateHistoryItemArray_size(instance->data); i++) {
        ProtoPirateHistoryItem* item = ProtoPirateHistoryItemArray_get(instance->data, i);
        furi_string_free(item->item_str);
        flipper_format_free(item->flipper_format);
        if(item->preset) {
            if(item->preset->name) {
                furi_string_free(item->preset->name);
            }
            free(item->preset);
        }
    }
    ProtoPirateHistoryItemArray_reset(instance->data);
    instance->last_index = 0;
}

uint16_t protopirate_history_get_item(ProtoPirateHistory* instance) {
    furi_assert(instance);
    return ProtoPirateHistoryItemArray_size(instance->data);
}

uint16_t protopirate_history_get_last_index(ProtoPirateHistory* instance) {
    furi_assert(instance);
    return instance->last_index;
}

// Helper function to free a single history item's resources
static void protopirate_history_item_free(ProtoPirateHistoryItem* item) {
    if(item->item_str) {
        furi_string_free(item->item_str);
        item->item_str = NULL;
    }
    if(item->flipper_format) {
        flipper_format_free(item->flipper_format);
        item->flipper_format = NULL;
    }
    if(item->preset) {
        if(item->preset->name) {
            furi_string_free(item->preset->name);
        }
        free(item->preset);
        item->preset = NULL;
    }
}

bool protopirate_history_add_to_history(
    ProtoPirateHistory* instance,
    void* context,
    SubGhzRadioPreset* preset,
    FuriString* history_item_str) {
    furi_assert(instance);
    furi_assert(context);

    SubGhzProtocolDecoderBase* decoder_base = context;

    // Check for duplicate (same hash within 500ms)
    if((instance->code_last_hash_data ==
        subghz_protocol_decoder_base_get_hash_data(decoder_base)) &&
       ((furi_get_tick() - instance->last_update_timestamp) < 500)) {
        instance->last_update_timestamp = furi_get_tick();
        return false;
    }

    FuriString* reuse_item_str = NULL;
    SubGhzRadioPreset* reuse_preset = NULL;

    // If history is full, remove the oldest entry
    if(ProtoPirateHistoryItemArray_size(instance->data) >= KIA_HISTORY_MAX) {
        ProtoPirateHistoryItem* oldest = ProtoPirateHistoryItemArray_get(instance->data, 0);
        if(oldest) {
            // Reuse resources to reduce heap churn
            reuse_item_str = oldest->item_str;
            oldest->item_str = NULL;

            reuse_preset = oldest->preset;
            oldest->preset = NULL;

            protopirate_history_item_free(oldest);
        }
        ProtoPirateHistoryItemArray_pop_at(NULL, instance->data, 0);
    }

    instance->code_last_hash_data = subghz_protocol_decoder_base_get_hash_data(decoder_base);
    instance->last_update_timestamp = furi_get_tick();

    // Create a new history item
    ProtoPirateHistoryItem* item = ProtoPirateHistoryItemArray_push_raw(instance->data);

    if(reuse_item_str) {
        item->item_str = reuse_item_str;
        furi_string_reset(item->item_str);
    } else {
        item->item_str = furi_string_alloc();
    }

    item->flipper_format = flipper_format_string_alloc();
    item->type = 0;

    // Copy preset
    if(reuse_preset) {
        item->preset = reuse_preset;
    } else {
        item->preset = malloc(sizeof(SubGhzRadioPreset));
        item->preset->name = furi_string_alloc();
    }

    item->preset->frequency = preset->frequency;
    furi_string_set(item->preset->name, preset->name);
    item->preset->data = preset->data;
    item->preset->data_size = preset->data_size;

    // Get string representation
    // Optimization: Write directly to item->item_str to avoid temporary allocation and copy
    furi_string_reset(item->item_str);
    if(history_item_str) {
        furi_string_set(item->item_str, history_item_str);
    } else {
        subghz_protocol_decoder_base_get_string(decoder_base, item->item_str);
    }

    // Serialize to flipper format
    subghz_protocol_decoder_base_serialize(decoder_base, item->flipper_format, preset);

    instance->last_index++;

    FURI_LOG_I(
        TAG,
        "Added item %u to history (size: %zu)",
        instance->last_index,
        ProtoPirateHistoryItemArray_size(instance->data));

    return true;
}

void protopirate_history_get_text_item_menu(
    ProtoPirateHistory* instance,
    FuriString* output,
    uint16_t idx) {
    furi_assert(instance);
    furi_assert(output);

    if(idx >= ProtoPirateHistoryItemArray_size(instance->data)) {
        furi_string_set(output, "---");
        return;
    }

    ProtoPirateHistoryItem* item = ProtoPirateHistoryItemArray_get(instance->data, idx);

    // Get just the first line for the menu
    const char* str = furi_string_get_cstr(item->item_str);
    const char* newline = strchr(str, '\r');
    if(newline) {
        size_t len = newline - str;
        furi_string_set_strn(output, str, len);
    } else {
        newline = strchr(str, '\n');
        if(newline) {
            size_t len = newline - str;
            furi_string_set_strn(output, str, len);
        } else {
            furi_string_set(output, item->item_str);
        }
    }
}

void protopirate_history_get_text_item(
    ProtoPirateHistory* instance,
    FuriString* output,
    uint16_t idx) {
    furi_assert(instance);
    furi_assert(output);

    if(idx >= ProtoPirateHistoryItemArray_size(instance->data)) {
        furi_string_set(output, "---");
        return;
    }

    ProtoPirateHistoryItem* item = ProtoPirateHistoryItemArray_get(instance->data, idx);
    furi_string_set(output, item->item_str);
}

SubGhzProtocolDecoderBase*
    protopirate_history_get_decoder_base(ProtoPirateHistory* instance, uint16_t idx) {
    UNUSED(instance);
    UNUSED(idx);
    return NULL;
}

FlipperFormat* protopirate_history_get_raw_data(ProtoPirateHistory* instance, uint16_t idx) {
    furi_assert(instance);

    if(idx >= ProtoPirateHistoryItemArray_size(instance->data)) {
        return NULL;
    }

    ProtoPirateHistoryItem* item = ProtoPirateHistoryItemArray_get(instance->data, idx);
    return item->flipper_format;
}
