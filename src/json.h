/**
 * @file json.h
 */
#ifndef CAMSIM_JSON_H
#define CAMSIM_JSON_H
#include "utils.h"
#include <stdio.h>
#include <stdlib.h>

constexpr int MAX_JSON_TOKENS = 1028;
constexpr int MAX_JSON_KEY_SIZE = 128;
constexpr int MAX_JSON_OBJECTS = 1028;

/**
 * All possible token types used by the JSON tokenizer.
 */
enum JsonTokenType
{
    JSON_TOKEN_NULL = 0,
    JSON_TOKEN_COLON,
    JSON_TOKEN_COMMA,
    JSON_TOKEN_QUOTE,
    JSON_TOKEN_OPEN_CURLY,
    JSON_TOKEN_CLOSE_CURLY,
    JSON_TOKEN_OPEN_SQUARE,
    JSON_TOKEN_CLOSE_SQUARE,
    JSON_TOKEN_IDENT,
    JSON_TOKEN_BOOLEAN,
    JSON_TOKEN_DOUBLE,
    JSON_TOKEN_INTEGER
};

/**
 * A JSON token fat struct.
 */
typedef struct
{
    /**
     * The type of token
     */
    JsonTokenType type;

    /**
     * The size of the whitespace (number of characters) if this is a whitespace token, else 0.
     */
    int whitespace_size;

    /**
     * The pointer to the ident if this is a ident token, else NULL
     */
    const String* ident_value;

    /**
     * The boolean value if this is a boolean token, else false
     */
    bool boolean_value;

    /**
     * The double value if this is a double token, else 0.0
     */
    double double_value;

    /**
     * The integer value if this is an integer token, else 0
     */
    long integer_value;
} JsonToken;

/**
 * A function that breaks a string into JSON tokens.
 *
 * @param[in] arena The arena you want to use for memory allocation.
 * @param[in] string A string containing the JSON you want to tokenize.
 *
 * @return A list of JSON tokens or NULL if the string cannot be tokenized.
 */
List* json_tokenize (Arena* arena, const String* string);

enum JsonObjectType
{
    JSON_OBJECT_NULL = 0,
    JSON_OBJECT_DICT = 1,
    JSON_OBJECT_LIST = 2,
    JSON_OBJECT_STRING = 3,
    JSON_OBJECT_BOOLEAN = 4,
    JSON_OBJECT_DOUBLE = 5,
    JSON_OBJECT_INTEGER = 6
};

typedef struct JsonObject JsonObject;

struct JsonObject
{
    /**
     * The type of the object.
     */
    JsonObjectType type;

    /**
     * The first key in a dictionary, NULL otherwise.
     */
    JsonObject* first_key;

    /**
     * The next key where the parent is a dictionary, NULL if the last key or if the parent is not a
     * dictionary.
     */
    JsonObject* next_key;

    /**
     * The first value in a dictionary, NULL otherwise.
     */
    JsonObject* first_value;

    /**
     * The next value where the parent is a dictionary or list, NULL if the last value or if parent
     * is not a list.
     */
    JsonObject* next_value;

    /**
     * The string value if the object is a string, NULL otherwise.
     */
    const String* string_value;

    /**
     * The boolean value if the object is a boolean, false otherwise.
     */
    bool boolean_value;

    /**
     * The double value if the object is a double, 0.0 otherwise.
     */
    double double_value;

    /**
     * The integer value if the object is an integer, 0 otherwise.
     */
    long integer_value;
};

/**
 * Parses tokenized JSON.
 *
 * @param[in] arena The arena you want to use for memory allocation.
 * @param[in] tokens The list of tokens returned from json_tokenize.
 *
 * @return A pointer to the first JSON object or NULL if parsing failed.
 */
JsonObject* json_parse_tokens (Arena* arena, List* tokens);

/**
 * Parses a string into JSON if it is valid.
 *
 * @param[in] arena The arena you want to use for memory allocation.
 * @param[in] string The string of text you want to parse into JSON
 *
 * @return A pointer to the first JSON object or NULL if tokenization or parsing failed.
 */
JsonObject* json_parse (Arena* arena, const String* string);

/**
 * Grabs a value from a JSON dictionary
 *
 * @param[in] dict A JSON object that is a dictionary.  Will return null if this is not a
 * dictionary.
 * @param[in] key The key you want to get.
 *
 * @return A pointer to the value if the key is in the dict, otherwise NULL.
 */
JsonObject* json_dictionary_get (JsonObject* dict, const String* key);

/**
 * Grabs a value from a JSON list
 *
 * @param[in] list A JSON object that is a list.  Will return null if this is not a list.
 * @param[in] index The index you want to get.  Bounds checking will happen.
 *
 * @return A pointer to the value if the index is valid, otherwise NULL.
 */
JsonObject* json_list_get (JsonObject* list, const int index);
#endif
