#include "json.h"
#include "utils.h"
#include <gtest/gtest.h>

class JsonTest : public ::testing::Test
{
  protected:
    Arena* arena = nullptr;

    void
    SetUp () override
    {
        // 1MB Arena for tests
        arena = arena_create (1028 * 1028);
        ASSERT_NE ((intptr_t)arena, (intptr_t)NULL);
    }

    void
    TearDown () override
    {
        arena_free (arena);
    }

    // Helper to streamline String creation during tests
    const String*
    MakeString (const char* str)
    {
        const String* s = string_create (arena, str);
        EXPECT_NE ((intptr_t)s, (intptr_t)NULL);
        return s;
    }
};

/* ========================================================================= *
 * json_tokenize
 * ========================================================================= */

TEST_F (JsonTest, TokenizeValidString)
{
    const String* string
        = MakeString ("{\"bingo\": 123.4,  \n\"bongo\": [\"ello\", 1, true, false, null]}");

    List* tokens = json_tokenize (arena, string);
    ASSERT_NE ((intptr_t)tokens, (intptr_t)NULL);
    ASSERT_EQ (tokens->size, 25u);

    EXPECT_EQ (((JsonToken*)list_get (tokens, 0))->type, JSON_TOKEN_OPEN_CURLY);
    EXPECT_EQ (((JsonToken*)list_get (tokens, 1))->type, JSON_TOKEN_QUOTE);
    EXPECT_EQ (((JsonToken*)list_get (tokens, 2))->type, JSON_TOKEN_IDENT);

    const String* bingo = MakeString ("bingo");
    EXPECT_EQ (string_compare (((JsonToken*)list_get (tokens, 2))->ident_value, bingo), 0);

    EXPECT_EQ (((JsonToken*)list_get (tokens, 3))->type, JSON_TOKEN_QUOTE);
    EXPECT_EQ (((JsonToken*)list_get (tokens, 4))->type, JSON_TOKEN_COLON);
    EXPECT_EQ (((JsonToken*)list_get (tokens, 5))->type, JSON_TOKEN_DOUBLE);
    EXPECT_NEAR (((JsonToken*)list_get (tokens, 5))->double_value, 123.4, 1e-14);
    EXPECT_EQ (((JsonToken*)list_get (tokens, 6))->type, JSON_TOKEN_COMMA);
    EXPECT_EQ (((JsonToken*)list_get (tokens, 7))->type, JSON_TOKEN_QUOTE);
    EXPECT_EQ (((JsonToken*)list_get (tokens, 8))->type, JSON_TOKEN_IDENT);

    const String* bongo = MakeString ("bongo");
    EXPECT_EQ (string_compare (((JsonToken*)list_get (tokens, 8))->ident_value, bongo), 0);

    EXPECT_EQ (((JsonToken*)list_get (tokens, 9))->type, JSON_TOKEN_QUOTE);
    EXPECT_EQ (((JsonToken*)list_get (tokens, 10))->type, JSON_TOKEN_COLON);
    EXPECT_EQ (((JsonToken*)list_get (tokens, 11))->type, JSON_TOKEN_OPEN_SQUARE);
    EXPECT_EQ (((JsonToken*)list_get (tokens, 12))->type, JSON_TOKEN_QUOTE);
    EXPECT_EQ (((JsonToken*)list_get (tokens, 13))->type, JSON_TOKEN_IDENT);

    const String* ello = MakeString ("ello");
    EXPECT_EQ (string_compare (((JsonToken*)list_get (tokens, 13))->ident_value, ello), 0);

    EXPECT_EQ (((JsonToken*)list_get (tokens, 14))->type, JSON_TOKEN_QUOTE);
    EXPECT_EQ (((JsonToken*)list_get (tokens, 15))->type, JSON_TOKEN_COMMA);
    EXPECT_EQ (((JsonToken*)list_get (tokens, 16))->type, JSON_TOKEN_INTEGER);
    EXPECT_EQ (((JsonToken*)list_get (tokens, 16))->integer_value, 1);
    EXPECT_EQ (((JsonToken*)list_get (tokens, 17))->type, JSON_TOKEN_COMMA);
    EXPECT_EQ (((JsonToken*)list_get (tokens, 18))->type, JSON_TOKEN_BOOLEAN);
    EXPECT_EQ (((JsonToken*)list_get (tokens, 18))->boolean_value, true);
    EXPECT_EQ (((JsonToken*)list_get (tokens, 19))->type, JSON_TOKEN_COMMA);
    EXPECT_EQ (((JsonToken*)list_get (tokens, 20))->type, JSON_TOKEN_BOOLEAN);
    EXPECT_EQ (((JsonToken*)list_get (tokens, 20))->boolean_value, false);
    EXPECT_EQ (((JsonToken*)list_get (tokens, 21))->type, JSON_TOKEN_COMMA);
    EXPECT_EQ (((JsonToken*)list_get (tokens, 22))->type, JSON_TOKEN_NULL);
    EXPECT_EQ (((JsonToken*)list_get (tokens, 23))->type, JSON_TOKEN_CLOSE_SQUARE);
    EXPECT_EQ (((JsonToken*)list_get (tokens, 24))->type, JSON_TOKEN_CLOSE_CURLY);
}

/* ========================================================================= *
 * json_parse_tokens
 * ========================================================================= */

TEST_F (JsonTest, ParseTokensValidSequence)
{
    const String* json_str = MakeString ("{\"id\": 42}");
    List* tokens = json_tokenize (arena, json_str);
    ASSERT_NE ((intptr_t)tokens, (intptr_t)NULL);

    JsonObject* root = json_parse_tokens (arena, tokens);
    ASSERT_NE ((intptr_t)root, (intptr_t)NULL);
    EXPECT_EQ (root->type, JSON_OBJECT_DICT);
}

TEST_F (JsonTest, ParseTokensNullReturnsNull)
{
    JsonObject* root = json_parse_tokens (arena, NULL);
    EXPECT_EQ ((intptr_t)root, (intptr_t)NULL);
}

/* ========================================================================= *
 * json_parse
 * ========================================================================= */

TEST_F (JsonTest, ParseValidPrimitiveAndNestedJson)
{
    const String* json_str = MakeString ("{"
                                         "  \"name\": \"CamSim\","
                                         "  \"version\": 2,"
                                         "  \"ratio\": 3.14,"
                                         "  \"active\": true,"
                                         "  \"deprecated\": false,"
                                         "  \"meta\": null,"
                                         "  \"tags\": [\"sim\", \"camera\"]"
                                         "}");

    JsonObject* root = json_parse (arena, json_str);
    ASSERT_NE ((intptr_t)root, (intptr_t)NULL);
    EXPECT_EQ (root->type, JSON_OBJECT_DICT);
}

TEST_F (JsonTest, ParseUnterminatedStringReturnsNull)
{
    // The tokenizer produces tokens for this input, but the parser must identify
    // the missing closing quote/delimiter and reject the JSON structure.
    const String* invalid_str = MakeString ("{\"key\": \"unterminated_string}");
    JsonObject* root = json_parse (arena, invalid_str);
    EXPECT_EQ ((intptr_t)root, (intptr_t)NULL);
}

TEST_F (JsonTest, ParseInvalidJsonReturnsNull)
{
    // Malformed JSON (missing closing brace)
    const String* invalid_str = MakeString ("{\"key\": 10");
    JsonObject* root = json_parse (arena, invalid_str);
    EXPECT_EQ ((intptr_t)root, (intptr_t)NULL);
}

/* ========================================================================= *
 * json_dictionary_get
 * ========================================================================= */

TEST_F (JsonTest, DictionaryGetValidKeys)
{
    const String* json_str = MakeString ("{"
                                         "  \"str_key\": \"hello\","
                                         "  \"int_key\": 100,"
                                         "  \"bool_key\": true,"
                                         "  \"null_key\": null"
                                         "}");

    JsonObject* root = json_parse (arena, json_str);
    ASSERT_NE ((intptr_t)root, (intptr_t)NULL);

    // String Value
    JsonObject* str_obj = json_dictionary_get (root, MakeString ("str_key"));
    ASSERT_NE ((intptr_t)str_obj, (intptr_t)NULL);
    EXPECT_EQ (str_obj->type, JSON_OBJECT_STRING);
    EXPECT_EQ (string_compare (str_obj->string_value, MakeString ("hello")), 0);

    // Integer Value
    JsonObject* int_obj = json_dictionary_get (root, MakeString ("int_key"));
    ASSERT_NE ((intptr_t)int_obj, (intptr_t)NULL);
    EXPECT_EQ (int_obj->type, JSON_OBJECT_INTEGER);
    EXPECT_EQ (int_obj->integer_value, 100);

    // Boolean Value
    JsonObject* bool_obj = json_dictionary_get (root, MakeString ("bool_key"));
    ASSERT_NE ((intptr_t)bool_obj, (intptr_t)NULL);
    EXPECT_EQ (bool_obj->type, JSON_OBJECT_BOOLEAN);
    EXPECT_TRUE (bool_obj->boolean_value);

    // Null Value
    JsonObject* null_obj = json_dictionary_get (root, MakeString ("null_key"));
    ASSERT_NE ((intptr_t)null_obj, (intptr_t)NULL);
    EXPECT_EQ (null_obj->type, JSON_OBJECT_NULL);
}

TEST_F (JsonTest, DictionaryGetNonExistentKeyReturnsNull)
{
    JsonObject* root = json_parse (arena, MakeString ("{\"a\": 1}"));
    ASSERT_NE ((intptr_t)root, (intptr_t)NULL);

    JsonObject* result = json_dictionary_get (root, MakeString ("non_existent"));
    EXPECT_EQ ((intptr_t)result, (intptr_t)NULL);
}

TEST_F (JsonTest, DictionaryGetOnNonDictReturnsNull)
{
    // Pass a List object instead of a Dictionary
    JsonObject* root = json_parse (arena, MakeString ("[1, 2, 3]"));
    ASSERT_NE ((intptr_t)root, (intptr_t)NULL);

    JsonObject* result = json_dictionary_get (root, MakeString ("key"));
    EXPECT_EQ ((intptr_t)result, (intptr_t)NULL);
}

TEST_F (JsonTest, DictionaryGetNullArgumentsReturnsNull)
{
    JsonObject* result = json_dictionary_get (NULL, MakeString ("key"));
    EXPECT_EQ ((intptr_t)result, (intptr_t)NULL);
}

/* ========================================================================= *
 * json_list_get
 * ========================================================================= */

TEST_F (JsonTest, ListGetValidIndices)
{
    const String* json_str = MakeString ("[10, 20.5, \"item\", false]");
    JsonObject* root = json_parse (arena, json_str);

    ASSERT_NE ((intptr_t)root, (intptr_t)NULL);
    EXPECT_EQ (root->type, JSON_OBJECT_LIST);

    // Index 0: Integer
    JsonObject* item0 = json_list_get (root, 0);
    ASSERT_NE ((intptr_t)item0, (intptr_t)NULL);
    EXPECT_EQ (item0->type, JSON_OBJECT_INTEGER);
    EXPECT_EQ (item0->integer_value, 10);

    // Index 1: Double
    JsonObject* item1 = json_list_get (root, 1);
    ASSERT_NE ((intptr_t)item1, (intptr_t)NULL);
    EXPECT_EQ (item1->type, JSON_OBJECT_DOUBLE);
    EXPECT_NEAR (item1->double_value, 20.5, 1e-14);

    // Index 2: String
    JsonObject* item2 = json_list_get (root, 2);
    ASSERT_NE ((intptr_t)item2, (intptr_t)NULL);
    EXPECT_EQ (item2->type, JSON_OBJECT_STRING);
    EXPECT_EQ (string_compare (item2->string_value, MakeString ("item")), 0);

    // Index 3: Boolean
    JsonObject* item3 = json_list_get (root, 3);
    ASSERT_NE ((intptr_t)item3, (intptr_t)NULL);
    EXPECT_EQ (item3->type, JSON_OBJECT_BOOLEAN);
    EXPECT_FALSE (item3->boolean_value);
}

TEST_F (JsonTest, ListGetOutOfBoundsReturnsNull)
{
    JsonObject* root = json_parse (arena, MakeString ("[1, 2]"));
    ASSERT_NE ((intptr_t)root, (intptr_t)NULL);

    // Negative index
    EXPECT_EQ ((intptr_t)json_list_get (root, -1), (intptr_t)NULL);

    // Exact length index (0-indexed, length is 2)
    EXPECT_EQ ((intptr_t)json_list_get (root, 2), (intptr_t)NULL);

    // Well past bounds
    EXPECT_EQ ((intptr_t)json_list_get (root, 99), (intptr_t)NULL);
}

TEST_F (JsonTest, ListGetOnNonListReturnsNull)
{
    // Pass a Dictionary object instead of a List
    JsonObject* root = json_parse (arena, MakeString ("{\"key\": \"val\"}"));
    ASSERT_NE ((intptr_t)root, (intptr_t)NULL);

    JsonObject* result = json_list_get (root, 0);
    EXPECT_EQ ((intptr_t)result, (intptr_t)NULL);
}

TEST_F (JsonTest, ListGetNullArgumentReturnsNull)
{
    JsonObject* result = json_list_get (NULL, 0);
    EXPECT_EQ ((intptr_t)result, (intptr_t)NULL);
}

/* ========================================================================= *
 * Complex / Nested Lookups
 * ========================================================================= */

TEST_F (JsonTest, NestedLookupCombination)
{
    const String* json_str = MakeString ("{"
                                         "  \"users\": ["
                                         "    {\"name\": \"Alice\", \"id\": 1},"
                                         "    {\"name\": \"Bob\", \"id\": 2}"
                                         "  ]"
                                         "}");

    JsonObject* root = json_parse (arena, json_str);
    ASSERT_NE ((intptr_t)root, (intptr_t)NULL);

    // Step 1: Get "users" list from root dict
    JsonObject* users_list = json_dictionary_get (root, MakeString ("users"));
    ASSERT_NE ((intptr_t)users_list, (intptr_t)NULL);
    EXPECT_EQ (users_list->type, JSON_OBJECT_LIST);

    // Step 2: Get 2nd element (Bob) from list
    JsonObject* bob_dict = json_list_get (users_list, 1);
    ASSERT_NE ((intptr_t)bob_dict, (intptr_t)NULL);
    EXPECT_EQ (bob_dict->type, JSON_OBJECT_DICT);

    // Step 3: Get "name" from Bob's dict
    JsonObject* bob_name = json_dictionary_get (bob_dict, MakeString ("name"));
    ASSERT_NE ((intptr_t)bob_name, (intptr_t)NULL);
    EXPECT_EQ (bob_name->type, JSON_OBJECT_STRING);
    EXPECT_EQ (string_compare (bob_name->string_value, MakeString ("Bob")), 0);
}
