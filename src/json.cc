/**
 * @file json.cc
 */
#include "json.h"

/*
 * Tokenization logic.
 */
List*
json_tokenize (Arena* arena, const String* string)
{
    if (string == NULL)
    {
        return NULL;
    }
    /*
     * These are used later, but creating here so we only allocate one of them.
     */
    const String* true_string = string_create (arena, "true");
    if (true_string == NULL)
    {
        return NULL;
    }
    const String* false_string = string_create (arena, "false");
    if (false_string == NULL)
    {
        return NULL;
    }
    const String* null_string = string_create (arena, "null");
    if (null_string == NULL)
    {
        return NULL;
    }

    /*
     * Create buffers for storage, and a way to know where we are in the buffers.
     */
    char* char_buffer
        = (char*)arena_multi_allocate (arena, MAX_JSON_KEY_SIZE + 1, sizeof (char), alignof (char));
    if (char_buffer == NULL)
    {
        return NULL;
    }
    List* tokens = list_create (arena, MAX_JSON_TOKENS, sizeof (JsonToken), alignof (JsonToken));
    if (tokens == NULL)
    {
        return NULL;
    }

    int char_buffer_position = 0;

    /*
     * Iterate through the string, breaking it into tokens.
     */
    size_t i;
    for (i = 0; i < string->size; i++)
    {
        /*
         * Fail if we run out of size.
         */
        if (char_buffer_position >= MAX_JSON_KEY_SIZE)
        {
            return NULL;
        }

        /*
         * Get current state.
         */
        char current_char = string->text[i];

        /*
         * Stuff is in the buffer.
         */
        if (char_buffer_position != 0)
        {
            const bool terminal_character = current_char == ':' or current_char == ','
                                            or current_char == '"' or current_char == '{'
                                            or current_char == '}' or current_char == '['
                                            or current_char == ']' or current_char == '\n';
            /*
             * We are getting to the end of an ident or number.
             */
            if (terminal_character)
            {
                /*
                 * Make the character buffer a C string so we can use C functions.
                 */
                char_buffer[char_buffer_position] = '\0';

                /*
                 * See if it is an integer.
                 */
                char* end_ptr = NULL;
                long integer_value = strtol (char_buffer, &end_ptr, 10);

                if (end_ptr == char_buffer + char_buffer_position)
                {
                    JsonToken token{};
                    token.type = JSON_TOKEN_INTEGER;
                    token.integer_value = integer_value;
                    if (list_append (tokens, &token) == NULL)
                    {
                        return NULL;
                    }
                }
                else
                {
                    /*
                     * Not an integer, see if it is a double.
                     */
                    double double_value = strtod (char_buffer, &end_ptr);
                    if (end_ptr == char_buffer + char_buffer_position)
                    {
                        JsonToken token{};
                        token.type = JSON_TOKEN_DOUBLE;
                        token.double_value = double_value;
                        if (list_append (tokens, &token) == NULL)
                        {
                            return NULL;
                        }
                    }
                    else
                    {
                        /*
                         * Not a double, see if it is a special type of string, or just a string.
                         */
                        const String* string = string_create (arena, char_buffer);
                        if (string == NULL)
                        {
                            return NULL;
                        }

                        if (string_compare (string, true_string) == 0)
                        {
                            JsonToken token{};
                            token.type = JSON_TOKEN_BOOLEAN;
                            token.boolean_value = true;
                            if (list_append (tokens, &token) == NULL)
                            {
                                return NULL;
                            }
                        }
                        else if (string_compare (string, false_string) == 0)
                        {
                            JsonToken token{};
                            token.type = JSON_TOKEN_BOOLEAN;
                            token.boolean_value = false;
                            if (list_append (tokens, &token) == NULL)
                            {
                                return NULL;
                            }
                        }
                        else if (string_compare (string, null_string) == 0)
                        {
                            JsonToken token{};
                            token.type = JSON_TOKEN_NULL;
                            if (list_append (tokens, &token) == NULL)
                            {
                                return NULL;
                            }
                        }
                        else
                        {
                            JsonToken token{};
                            token.type = JSON_TOKEN_IDENT;
                            token.ident_value = string;
                            if (list_append (tokens, &token) == NULL)
                            {
                                return NULL;
                            }
                        }
                    }
                }
                char_buffer_position = 0;
            }
        }

        /*
         * Handle the current character
         */
        const bool is_lowercase_letter = 'a' <= current_char and current_char <= 'z';
        const bool is_uppercase_letter = 'A' <= current_char and current_char <= 'Z';
        const bool is_number = '0' <= current_char and current_char <= '9';
        if (current_char == ':')
        {
            JsonToken token{};
            token.type = JSON_TOKEN_COLON;
            if (list_append (tokens, &token) == NULL)
            {
                return NULL;
            }
        }
        else if (current_char == ',')
        {
            JsonToken token{};
            token.type = JSON_TOKEN_COMMA;
            if (list_append (tokens, &token) == NULL)
            {
                return NULL;
            }
        }
        else if (current_char == '"')
        {
            JsonToken token{};
            token.type = JSON_TOKEN_QUOTE;
            if (list_append (tokens, &token) == NULL)
            {
                return NULL;
            }
        }
        else if (current_char == '{')
        {
            JsonToken token{};
            token.type = JSON_TOKEN_OPEN_CURLY;
            if (list_append (tokens, &token) == NULL)
            {
                return NULL;
            }
        }
        else if (current_char == '}')
        {
            JsonToken token{};
            token.type = JSON_TOKEN_CLOSE_CURLY;
            if (list_append (tokens, &token) == NULL)
            {
                return NULL;
            }
        }
        else if (current_char == '[')
        {
            JsonToken token{};
            token.type = JSON_TOKEN_OPEN_SQUARE;
            if (list_append (tokens, &token) == NULL)
            {
                return NULL;
            }
        }
        else if (current_char == ']')
        {
            JsonToken token{};
            token.type = JSON_TOKEN_CLOSE_SQUARE;
            if (list_append (tokens, &token) == NULL)
            {
                return NULL;
            }
        }
        else if (is_lowercase_letter or is_uppercase_letter or is_number or current_char == '_'
                 or current_char == '-' or current_char == '.')
        {
            char_buffer[char_buffer_position] = current_char;
            char_buffer_position++;
        }
    }

    if (i != string->size)
    {
        return NULL;
    }

    return tokens;
}

/*
 * Parsing logic.  Predefine all functions here and not in the header because they are private.
 */

JsonObject* json_dictionary (Arena* arena, List* tokens, int* parse_idx);
JsonObject* json_list (Arena* arena, List* tokens, int* parse_idx);
JsonObject* json_string (Arena* arena, List* tokens, int* parse_idx);
JsonObject* json_value (Arena* arena, List* tokens, int* parse_idx);

/**
 * Used to check for a specific token at parse_idx.
 *
 * @param[in] type The type of token we want.
 * @param[in] tokens The list of tokens.
 * @param[inout] parse_idx The current parse position.  Is updated if we have the correct token.
 *
 * @return A pointer to the current token if there is a match, otherwise NULL.
 */
JsonToken*
parse_expect (JsonTokenType type, List* tokens, int* parse_idx)
{
    JsonToken* current_token = (JsonToken*)list_get (tokens, *parse_idx);
    if (current_token == NULL)
    {
        return NULL;
    }

    if (current_token->type != type)
    {
        return NULL;
    }

    (*parse_idx)++;
    return current_token;
}

JsonObject*
json_dictionary (Arena* arena, List* tokens, int* parse_idx)
{
    int starting_idx = *parse_idx;

    /*
     * Check for opening curly bracket.
     */
    JsonToken* open_curly = parse_expect (JSON_TOKEN_OPEN_CURLY, tokens, parse_idx);
    if (open_curly == NULL)
    {
        *parse_idx = starting_idx;
        return NULL;
    }

    /*
     * Get items in dictionary, starting with first set, and then using a comma to separate all
     * additional sets.
     */
    JsonObject* first_key = NULL;
    JsonObject* first_value = NULL;
    JsonObject* prev_key = NULL;
    JsonObject* prev_value = NULL;
    int items_idx = *parse_idx;
    if ((first_key = json_string (arena, tokens, parse_idx)) != NULL
        and parse_expect (JSON_TOKEN_COLON, tokens, parse_idx) != NULL
        and (first_value = json_value (arena, tokens, parse_idx)) != NULL)
    {
        /*
         * If we get here, then we have at least one set of items.  This is a known good point, so
         * we can update items_idx to the current parse_idx.
         */
        items_idx = *parse_idx;

        /*
         * We are going to create a linked list of JsonObjects for the keys and the values for each
         * additional set that is added.  Initialize the linked lists with the first items, and then
         * look for more.
         */
        prev_key = first_key;
        prev_value = first_value;
        JsonObject* next_key = NULL;
        JsonObject* next_value = NULL;
        while (parse_expect (JSON_TOKEN_COMMA, tokens, parse_idx) != NULL
               and (next_key = json_string (arena, tokens, parse_idx)) != NULL
               and parse_expect (JSON_TOKEN_COLON, tokens, parse_idx) != NULL
               and (next_value = json_value (arena, tokens, parse_idx)) != NULL)
        {
            /*
             * If we get to this point, we know we have another set of items.  Update items idx
             * since we are in a known good point, and then populate the linked list.
             */
            items_idx = *parse_idx;
            prev_key->next_key = next_key;
            prev_value->next_value = next_value;
            prev_key = next_key;
            prev_value = next_value;
        }
    }
    /*
     * Since items_idx is always at a known good point, we can reset it at the end, and if we just
     * had a successful parse, then it will not move.
     */
    *parse_idx = items_idx;

    /*
     * Check for a closing curly bracket.
     */
    JsonToken* close_curly = parse_expect (JSON_TOKEN_CLOSE_CURLY, tokens, parse_idx);
    if (close_curly == NULL)
    {
        *parse_idx = starting_idx;
        return NULL;
    }

    /*
     * We have a valid dictionary at this point.  Create one, fill it with the items, and return.
     */
    JsonObject* dictionary = arena_allocate_type (arena, JsonObject);
    if (dictionary == NULL)
    {
        return NULL;
    }
    dictionary->type = JSON_OBJECT_DICT;
    dictionary->first_key = first_key;
    dictionary->first_value = first_value;
    return dictionary;
}

JsonObject*
json_list (Arena* arena, List* tokens, int* parse_idx)
{
    int starting_idx = *parse_idx;

    /*
     * Check for opening square bracket.
     */
    JsonToken* open_square = parse_expect (JSON_TOKEN_OPEN_SQUARE, tokens, parse_idx);
    if (open_square == NULL)
    {
        *parse_idx = starting_idx;
        return NULL;
    }

    /*
     * Get values in list, starting with first value, and then using a comma to separate all
     * additional values.
     */
    JsonObject* first_value = NULL;
    JsonObject* prev_value = NULL;
    int values_idx = *parse_idx;
    if ((first_value = json_value (arena, tokens, parse_idx)) != NULL)
    {
        /*
         * If we get here, then we have at least one value.  This is a known good point, so we can
         * update values_idx to the current parse_idx.
         */
        values_idx = *parse_idx;

        /*
         * We are going to create a linked list of JsonObjects for the values for each additional
         * one that is added.  Initialize the linked lists with the first items, and then look for
         * more.
         */
        prev_value = first_value;
        JsonObject* next_value = NULL;
        while (parse_expect (JSON_TOKEN_COMMA, tokens, parse_idx) != NULL
               and (next_value = json_value (arena, tokens, parse_idx)) != NULL)
        {
            /*
             * If we get to this point, we know we have another value.  Update items idx since we
             * are in a known good point, and then populate the linked list.
             */
            values_idx = *parse_idx;
            prev_value->next_value = next_value;
            prev_value = next_value;
        }
    }
    /*
     * Since values_idx is always at a known good point, we can reset it at the end, and if we just
     * had a successful parse, then it will not move.
     */
    *parse_idx = values_idx;

    /*
     * Check for a closing square bracket.
     */
    JsonToken* close_square = parse_expect (JSON_TOKEN_CLOSE_SQUARE, tokens, parse_idx);
    if (close_square == NULL)
    {
        *parse_idx = starting_idx;
        return NULL;
    }

    /*
     * We have a valid list at this point.  Create one, fill it with the items, and return.
     */
    JsonObject* list = arena_allocate_type (arena, JsonObject);
    if (list == NULL)
    {
        return NULL;
    }
    list->type = JSON_OBJECT_LIST;
    list->first_value = first_value;
    return list;
}

JsonObject*
json_string (Arena* arena, List* tokens, int* parse_idx)
{
    int starting_idx = *parse_idx;

    /*
     * Check for opening quote.
     */
    JsonToken* open_quote = parse_expect (JSON_TOKEN_QUOTE, tokens, parse_idx);
    if (open_quote == NULL)
    {
        *parse_idx = starting_idx;
        return NULL;
    }

    /*
     * Create a special arena for all of the little strings in this function that we don't need
     * outside of this function.  Then create those strings.
     */
    Arena* helper_string_arena = arena_create (MAX_JSON_TOKENS * MAX_JSON_KEY_SIZE);
    if (helper_string_arena == NULL)
    {
        return NULL;
    }
    const String* null = string_create (helper_string_arena, "null");
    if (null == NULL)
    {
        arena_free (helper_string_arena);
        return NULL;
    }
    const String* colon = string_create (helper_string_arena, ":");
    if (colon == NULL)
    {
        arena_free (helper_string_arena);
        return NULL;
    }
    const String* comma = string_create (helper_string_arena, ",");
    if (comma == NULL)
    {
        arena_free (helper_string_arena);
        return NULL;
    }
    const String* open_curly = string_create (helper_string_arena, "{");
    if (open_curly == NULL)
    {
        arena_free (helper_string_arena);
        return NULL;
    }
    const String* close_curly = string_create (helper_string_arena, "}");
    if (close_curly == NULL)
    {
        arena_free (helper_string_arena);
        return NULL;
    }
    const String* open_square = string_create (helper_string_arena, "[");
    if (open_square == NULL)
    {
        arena_free (helper_string_arena);
        return NULL;
    }
    const String* close_square = string_create (helper_string_arena, "]");
    if (close_square == NULL)
    {
        arena_free (helper_string_arena);
        return NULL;
    }
    const String* true_str = string_create (helper_string_arena, "true");
    if (true_str == NULL)
    {
        arena_free (helper_string_arena);
        return NULL;
    }
    const String* false_str = string_create (helper_string_arena, "false");
    if (false_str == NULL)
    {
        arena_free (helper_string_arena);
        return NULL;
    }

    JsonToken* string_token = NULL;
    const String* string_value = string_create (arena, "");
    char* char_buffer = NULL;
    bool quote_found = false;
    bool early_exit = false;
    while (not quote_found and not early_exit
           and (string_token = (JsonToken*)list_get (tokens, *parse_idx)) != NULL)
    {
        switch (string_token->type)
        {

        /*
         * Put close quote first just so we see it easier
         */
        case JSON_TOKEN_QUOTE:
            quote_found = true;
            break;

        /*
         * Go through each token type and perform string concatenation to get the final string.
         */
        case JSON_TOKEN_NULL:
            string_value = string_concatenate (arena, string_value, null);
            break;
        case JSON_TOKEN_COLON:
            string_value = string_concatenate (arena, string_value, colon);
            break;
        case JSON_TOKEN_COMMA:
            string_value = string_concatenate (arena, string_value, comma);
            break;
        case JSON_TOKEN_OPEN_CURLY:
            string_value = string_concatenate (arena, string_value, open_curly);
            break;
        case JSON_TOKEN_CLOSE_CURLY:
            string_value = string_concatenate (arena, string_value, close_curly);
            break;
        case JSON_TOKEN_OPEN_SQUARE:
            string_value = string_concatenate (arena, string_value, open_square);
            break;
        case JSON_TOKEN_CLOSE_SQUARE:
            string_value = string_concatenate (arena, string_value, close_square);
            break;
        case JSON_TOKEN_IDENT:
            string_value = string_concatenate (arena, string_value, string_token->ident_value);
            break;
        case JSON_TOKEN_BOOLEAN:
            string_value = string_concatenate (arena, string_value,
                                               string_token->boolean_value ? true_str : false_str);
            break;
        case JSON_TOKEN_DOUBLE:
            char_buffer = arena_multi_allocate_type (helper_string_arena, MAX_JSON_KEY_SIZE, char);
            if (char_buffer == NULL)
            {
                early_exit = true;
                break;
            }
            sprintf (char_buffer, "%lf", string_token->double_value);
            string_value = string_concatenate (arena, string_value,
                                               string_create (helper_string_arena, char_buffer));
            break;
        case JSON_TOKEN_INTEGER:
            char_buffer = arena_multi_allocate_type (helper_string_arena, MAX_JSON_KEY_SIZE, char);
            if (char_buffer == NULL)
            {
                early_exit = true;
                break;
            }
            sprintf (char_buffer, "%ld", string_token->integer_value);
            string_value = string_concatenate (arena, string_value,
                                               string_create (helper_string_arena, char_buffer));
            break;
        }
        (*parse_idx)++;
    }

    /*
     * Free the helper arena.
     */
    arena_free (helper_string_arena);

    /*
     * Ran out of tokens as string was never closed, or memory ran out.  Fail parse.
     */
    if (early_exit or string_token == NULL or string_token->type != JSON_TOKEN_QUOTE
        or not quote_found)
    {
        *parse_idx = starting_idx;
        return NULL;
    }

    /*
     * Successful parse!  Create a string JSON object and return.
     */
    JsonObject* string = arena_allocate_type (arena, JsonObject);
    if (string == NULL)
    {
        return NULL;
    }
    string->type = JSON_OBJECT_STRING;
    string->string_value = string_value;
    return string;
}

JsonObject*
json_value (Arena* arena, List* tokens, int* parse_idx)
{
    if (tokens == NULL)
    {
        return NULL;
    }
    JsonObject* value = NULL;
    JsonToken* token = NULL;

    if ((value = json_dictionary (arena, tokens, parse_idx)) != NULL)
    {
        /* Do nothing, as value is already allocated and populated. */
    }
    else if ((value = json_list (arena, tokens, parse_idx)) != NULL)
    {
        /* Do nothing, as value is already allocated and populated. */
    }
    else if ((value = json_string (arena, tokens, parse_idx)) != NULL)
    {
        /* Do nothing, as value is already allocated and populated. */
    }
    else if ((token = parse_expect (JSON_TOKEN_NULL, tokens, parse_idx)) != NULL)
    {
        value = arena_allocate_type (arena, JsonObject);
        if (value == NULL)
        {
            return NULL;
        }
        value->type = JSON_OBJECT_NULL;
    }
    else if ((token = parse_expect (JSON_TOKEN_BOOLEAN, tokens, parse_idx)) != NULL)
    {
        value = arena_allocate_type (arena, JsonObject);
        if (value == NULL)
        {
            return NULL;
        }
        value->type = JSON_OBJECT_BOOLEAN;
        value->boolean_value = token->boolean_value;
    }
    else if ((token = parse_expect (JSON_TOKEN_DOUBLE, tokens, parse_idx)) != NULL)
    {
        value = arena_allocate_type (arena, JsonObject);
        if (value == NULL)
        {
            return NULL;
        }
        value->type = JSON_OBJECT_DOUBLE;
        value->double_value = token->double_value;
    }
    else if ((token = parse_expect (JSON_TOKEN_INTEGER, tokens, parse_idx)) != NULL)
    {
        value = arena_allocate_type (arena, JsonObject);
        if (value == NULL)
        {
            return NULL;
        }
        value->type = JSON_OBJECT_INTEGER;
        value->integer_value = token->integer_value;
    }

    /*
     * Value will either be populated or NULL if parsing failed.  In either case, we can safely
     * return.
     */
    return value;
}

JsonObject*
json_parse_tokens (Arena* arena, List* tokens)
{
    int parse_idx = 0;

    return json_value (arena, tokens, &parse_idx);
}

JsonObject*
json_parse (Arena* arena, const String* string)
{
    List* tokens = json_tokenize (arena, string);
    if (tokens == NULL)
    {
        return NULL;
    }

    return json_parse_tokens (arena, tokens);
}

JsonObject*
json_dictionary_get (JsonObject* dict, const String* key)
{
    if (dict == NULL)
    {
        return NULL;
    }
    if (key == NULL)
    {
        return NULL;
    }
    if (dict->type != JSON_OBJECT_DICT)
    {
        return NULL;
    }

    JsonObject* current_key = dict->first_key;
    JsonObject* current_value = dict->first_value;
    JsonObject* desired_value = NULL;

    while (current_key != NULL and current_value != NULL)
    {
        if (current_key->type != JSON_OBJECT_STRING)
        {
            break;
        }
        if (string_compare (key, current_key->string_value) == 0)
        {
            desired_value = current_value;
            break;
        }
        current_key = current_key->next_key;
        current_value = current_value->next_value;
    }

    return desired_value;
}

JsonObject*
json_list_get (JsonObject* list, const int index)
{
    if (list == NULL)
    {
        return NULL;
    }
    if (list->type != JSON_OBJECT_LIST)
    {
        return NULL;
    }

    JsonObject* current_value = list->first_value;
    int current_index = 0;
    while (current_value != NULL and current_index <= index)
    {
        if (current_index == index)
        {
            return current_value;
        }
        current_value = current_value->next_value;
        current_index++;
    }

    return NULL;
}
