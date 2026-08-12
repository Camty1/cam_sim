#include "tools/cpp/runfiles/runfiles.h"
#include "utils.h"
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <gtest/gtest.h>
#include <stdio.h>

#define ASSERT_NOT_NULL(x) ASSERT_NE (x, nullptr)
#define EXPECT_NULL(x) EXPECT_EQ (x, nullptr)
#define EXPECT_NOT_NULL(x) EXPECT_NE (x, nullptr)

TEST (arena, test_capacity)
{
    Arena* arena = arena_create (2 * sizeof (int32_t));

    int32_t* valid_pointer = (int32_t*)arena_allocate (arena, sizeof (int32_t), alignof (int32_t));
    int64_t* invalid_pointer
        = (int64_t*)arena_allocate (arena, sizeof (int64_t), alignof (int64_t));

    EXPECT_NOT_NULL (valid_pointer);
    EXPECT_NULL (invalid_pointer);

    arena_free (arena);
}

TEST (arena, test_alignment)
{
    Arena* arena = arena_create (1028);

    int64_t* valid_pointer = (int64_t*)arena_allocate (arena, sizeof (int64_t), alignof (int64_t));
    int32_t* second_valid_pointer
        = (int32_t*)arena_allocate (arena, sizeof (int32_t), alignof (int32_t));
    int64_t* maybe_aligned_pointer
        = (int64_t*)arena_allocate (arena, sizeof (int64_t), alignof (int64_t));

    ASSERT_NOT_NULL (valid_pointer);
    ASSERT_NOT_NULL (second_valid_pointer);
    ASSERT_NOT_NULL (maybe_aligned_pointer);

    EXPECT_EQ ((uintptr_t)valid_pointer % alignof (int64_t), 0u);
    EXPECT_EQ ((uintptr_t)second_valid_pointer % alignof (int32_t), 0u);
    EXPECT_EQ ((uintptr_t)maybe_aligned_pointer % alignof (int64_t), 0u);

    EXPECT_EQ ((uintptr_t)second_valid_pointer + alignof (int64_t),
               (uintptr_t)maybe_aligned_pointer);

    arena_free (arena);
}

TEST (arena, test_multi_allocation)
{
    Arena* arena = arena_create (1028);

    size_t number_of_objects = 6;
    int64_t* valid_pointer = (int64_t*)arena_multi_allocate (arena, number_of_objects,
                                                             sizeof (int64_t), alignof (int64_t));
    char* second_valid_pointer = (char*)arena_allocate (arena, sizeof (char), alignof (char));

    ASSERT_NOT_NULL (valid_pointer);
    ASSERT_NOT_NULL (second_valid_pointer);

    EXPECT_EQ ((uintptr_t)valid_pointer + number_of_objects * sizeof (int64_t),
               (uintptr_t)second_valid_pointer);
}

TEST (string, test_string_creation)
{
    Arena* arena = arena_create (2 * MAX_STRING_SIZE);

    /*
     * Handles an empty string.
     */
    char empty_c_str[1] = "";
    const String* empty_string = string_create (arena, empty_c_str);
    EXPECT_EQ (empty_string->size, 0u);

    /*
     * Handles a normal string and copies text correctly.
     */
    char normal_c_str[13] = "Hello world!";
    const String* normal_string = string_create (arena, normal_c_str);
    EXPECT_EQ (normal_string->size, 12u);
    EXPECT_EQ (normal_string->text[0], 'H');
    EXPECT_EQ (normal_string->text[6], 'w');
    EXPECT_EQ (normal_string->text[normal_string->size - 1], '!');

    /*
     * Will not allocate a string that is too big.
     */
    size_t too_long_string_size = 1312;
    char too_long_c_str[1312]
        = "Lorem ipsum dolor sit amet, consectetur adipiscing elit, sed do eiusmod tempor "
          "incididunt ut labore et dolore magna aliqua. Ut enim ad minim veniam, quis nostrud "
          "exercitation ullamco laboris nisi ut aliquip ex ea commodo consequat. Duis aute irure "
          "dolor in reprehenderit in voluptate velit esse cillum dolore eu fugiat nulla pariatur. "
          "Excepteur sint occaecat cupidatat non proident, sunt in culpa qui officia deserunt "
          "mollit anim id est laborum. Sed ut perspiciatis unde omnis iste natus error sit "
          "voluptatem accusantium doloremque laudantium, totam rem aperiam, eaque ipsa quae ab "
          "illo inventore veritatis et quasi architecto beatae vitae dicta sunt explicabo. Nemo "
          "enim ipsam voluptatem quia voluptas sit aspernatur aut odit aut fugit, sed quia "
          "consequuntur magni dolores eos qui ratione voluptatem sequi nesciunt. Neque porro "
          "quisquam est, qui dolorem ipsum quia dolor sit amet, consectetur, adipisci velit, sed "
          "quia non numquam eius modi tempora incidunt ut labore et dolore magnam aliquam quaerat "
          "voluptatem. Ut enim ad minima veniam, quis nostrum exercitationem ullam corporis "
          "suscipit laboriosam, nisi ut aliquid ex ea commodi consequatur? Quis autem vel eum iure "
          "reprehenderit qui in ea voluptate velit esse quam nihil molestiae consequatur, vel "
          "illum qui dolorem eum fugiat quo voluptas nulla pariatur?";
    ASSERT_GT (too_long_string_size, (size_t)MAX_STRING_SIZE); /* Check that the test is still valid */
    const String* too_long_string = string_create (arena, too_long_c_str);
    EXPECT_NULL (too_long_string);

    /*
     * If the arena is full, then it will fail to allocate the string.
     */
    Arena* small_arena = arena_create (20);
    char full_arena_c_str[25] = "Bingo bongo bongo bingo.";
    const String* full_arena_string = string_create (small_arena, full_arena_c_str);
    EXPECT_NULL (full_arena_string);

    arena_free (arena);
    arena_free (small_arena);
}

TEST (string, test_string_c_str)
{
    Arena* arena = arena_create (1028);

    /*
     * Check that c_str -> String -> c_str = c_str
     */
    char hello[6] = "hello";
    const String* hello_string = string_create (arena, hello);
    char* test_hello = string_c_str (arena, hello_string);
    EXPECT_EQ (strcmp (hello, test_hello), 0);

    /*
     * Check empty strings work too
     */

    char empty[1] = "";
    const String* empty_string = string_create (arena, empty);
    char* test_empty = string_c_str (arena, empty_string);
    EXPECT_EQ (strcmp (empty, test_empty), 0);

    arena_free (arena);
}

TEST (string, test_string_compare)
{
    Arena* arena = arena_create (1028);

    /*
     * Test standard string compare of same length.
     */
    const String* abc = string_create (arena, "abc");
    const String* bcd = string_create (arena, "bcd");
    EXPECT_EQ (string_compare (abc, bcd), -1);
    EXPECT_EQ (string_compare (bcd, abc), 1);

    /*
     * Test matching strings.
     */
    const String* abc_2 = string_create (arena, "abc");
    EXPECT_EQ (string_compare (abc, abc), 0);
    EXPECT_EQ (string_compare (abc, abc_2), 0);

    /*
     * Test one string longer than the other.
     */
    const String* abcd = string_create (arena, "abcd");
    EXPECT_EQ (string_compare (abc, abcd), -1);
    EXPECT_EQ (string_compare (abcd, abc), 1);

    /*
     * Test empty string.
     */
    const String* empty = string_create (arena, "");
    EXPECT_EQ (string_compare (empty, abc), -1);
    EXPECT_EQ (string_compare (abc, empty), 1);
    EXPECT_EQ (string_compare (empty, empty), 0);

    arena_free (arena);
}

TEST (string, test_string_concatenate)
{
    Arena* arena = arena_create (4 * MAX_STRING_SIZE);

    /*
     * Normal concatenation.
     */
    const String* hello = string_create (arena, "Hello");
    const String* world = string_create (arena, " world!");
    const String* concatenated = string_concatenate (arena, hello, world);
    EXPECT_EQ (string_compare (concatenated, string_create (arena, "Hello world!")), 0);

    /*
     * Resulting concatenated string exceeds max string size.
     */
    const char long_c_str[MAX_STRING_SIZE]
        = "Lorem ipsum dolor sit amet, consectetur adipiscing elit, sed do eiusmod tempor "
          "incididunt ut labore et dolore magna aliqua. Ut enim ad minim veniam, quis nostrud "
          "exercitation ullamco laboris nisi ut aliquip ex ea commodo consequat. Duis aute irure "
          "dolor in reprehenderit in voluptate velit esse cillum dolore eu fugiat nulla pariatur. "
          "Excepteur sint occaecat cupidatat non proident, sunt in culpa qui officia deserunt "
          "mollit anim id est laborum. Sed ut perspiciatis unde omnis iste natus error sit "
          "voluptatem accusantium doloremque laudantium, totam rem aperiam, eaque ipsa quae ab "
          "illo inventore veritatis et quasi architecto beatae vitae dicta sunt explicabo. Nemo "
          "enim ipsam voluptatem quia voluptas sit aspernatur aut odit aut fugit, sed quia "
          "consequuntur magni dolores eos qui ratione voluptatem sequi nesciunt. Neque porro "
          "quisquam est, qui dolorem ipsum quia dolor sit amet, consectetur, adipisci velit, sed "
          "quia non numquam eius modi tempora incidunt ut labore et dolore magnam aliquam quaerat "
          "voluptatem. Ut enim ad mi";
    const String* long_string = string_create (arena, long_c_str);
    const char not_as_long_c_str[31] = "Hello world, my name is bingo!";
    const String* not_as_long_string = string_create (arena, not_as_long_c_str);
    const String* failed_concat = string_concatenate (arena, long_string, not_as_long_string);
    EXPECT_NULL (failed_concat);

    /*
     * Concatenation with empty string.
     */
    const String* bingo = string_create (arena, "bingo");
    const String* empty = string_create (arena, "");
    EXPECT_EQ (string_compare (string_concatenate (arena, bingo, empty), bingo), 0);
    EXPECT_EQ (string_compare (string_concatenate (arena, empty, bingo), bingo), 0);

    arena_free (arena);
}

const String*
get_runfiles_path (Arena* arena, const String* filename)
{
    /*
     * Gross C++ :(
     */
    using bazel::tools::cpp::runfiles::Runfiles;
    std::string error;
    static std::unique_ptr<Runfiles> runfiles (Runfiles::Create ("", &error));

    if (!runfiles)
    {
        fprintf (stderr, "Failed to init Bazel runfiles: %s\n", error.c_str ());
        return nullptr;
    }

    /*
     * Get full path
     */
    const String* prefix = string_create (arena, "camsim/dat/utils_test/");
    if (prefix == NULL)
    {
        return NULL;
    }
    const String* path = string_concatenate (arena, prefix, filename);
    if (path == NULL)
    {
        return NULL;
    }
    char* path_c_str = string_c_str (arena, path);
    if (path_c_str == NULL)
    {
        return NULL;
    }

    /*
     * Get runfiles path
     */
    std::string runfiles_path = runfiles->Rlocation (path_c_str);
    if (runfiles_path.empty ())
    {
        fprintf (stderr, "Bazel could not resolve path for: %s\n", path_c_str);
        return nullptr;
    }

    /*
     * We can just return here, since if the alloc fails, then it will return NULL
     */
    return string_create (arena, runfiles_path.c_str ());
}

TEST (string, test_string_file_read)
{
    Arena* arena = arena_create (6 * MAX_STRING_SIZE);

    /*
     * A file that will fit
     */
    const String* just_right_path
        = get_runfiles_path (arena, string_create (arena, "just_right.txt"));
    char* just_right_path_c_str = string_c_str (arena, just_right_path);
    ASSERT_NOT_NULL (just_right_path);
    ASSERT_NOT_NULL (just_right_path_c_str);

    FILE* just_right_file = fopen (just_right_path_c_str, "r");
    ASSERT_NOT_NULL (just_right_file);

    const String* just_right_contents = string_file_read (arena, just_right_file);
    fclose (just_right_file);

    EXPECT_NOT_NULL (just_right_contents);
    EXPECT_EQ (just_right_contents->text[0], 'H');
    EXPECT_EQ (just_right_contents->text[just_right_contents->size - 1], '\n');

    /*
     * A file that does not exist
     */
    FILE* missing_file = NULL;
    const String* missing_file_contents = string_file_read (arena, missing_file);

    EXPECT_NULL (missing_file_contents);

    /*
     * A file that is too long
     */
    const String* too_long_path = get_runfiles_path (arena, string_create (arena, "too_long.txt"));
    char* too_long_path_c_str = string_c_str (arena, too_long_path);
    ASSERT_NOT_NULL (too_long_path);
    ASSERT_NOT_NULL (too_long_path_c_str);

    FILE* too_long_file = fopen (too_long_path_c_str, "r");
    ASSERT_NOT_NULL (too_long_file);

    const String* too_long_contents = string_file_read (arena, too_long_file);
    fclose (too_long_file);

    EXPECT_NULL (too_long_contents);

    /*
     * An empty file
     */
    const String* empty_path = get_runfiles_path (arena, string_create (arena, "empty.txt"));
    char* empty_path_c_str = string_c_str (arena, empty_path);
    ASSERT_NOT_NULL (empty_path);
    ASSERT_NOT_NULL (empty_path_c_str);

    FILE* empty_file = fopen (empty_path_c_str, "r");
    ASSERT_NOT_NULL (empty_file);

    const String* empty_contents = string_file_read (arena, empty_file);
    fclose (empty_file);

    EXPECT_NOT_NULL (empty_contents);
    EXPECT_EQ (empty_contents->size, 0u);

    arena_free (arena);
}

TEST (array, test_array_create)
{
    Arena* arena = arena_create (1028);

    /*
     * Normal allocation.
     */
    Array* normal = array_create (arena, 10, sizeof (int), alignof (int));
    EXPECT_NOT_NULL (normal);

    /*
     * Too big.
     */
    Array* too_big = array_create (arena, 2000, sizeof (char), alignof (char));
    EXPECT_NULL (too_big);

    arena_free (arena);
}

TEST (array, test_array_get)
{
    Arena* arena = arena_create (1028);
    ASSERT_NOT_NULL (arena);

    Array* array = array_create (arena, 10, sizeof (int), alignof (int));
    ASSERT_NOT_NULL (array);

    /*
     * Being naughty and setting the values manually.
     */
    int* array_data = (int*)(array->data);
    *array_data = 10;
    *(array_data + 3) = 3;
    *(array_data + 9) = 9;

    /*
     * Get the first element.
     */
    int* first = (int*)array_get (array, 0);

    EXPECT_EQ (*first, 10);

    /*
     * Get a different element.
     */
    int* fourth = (int*)array_get (array, 3);
    EXPECT_EQ (*fourth, 3);

    /*
     * Get the last element.
     */
    int* last = (int*)array_get (array, -1);
    EXPECT_EQ (*last, 9);

    /*
     * Exceed the bounds.
     */
    int* bad = (int*)array_get (array, 10);
    EXPECT_NULL (bad);

    /*
     * Exceed the bounds in the other way.
     */
    int* negative_bad = (int*)array_get (array, -11);
    EXPECT_NULL (negative_bad);

    arena_free (arena);
}

TEST (array, test_array_set)
{
    Arena* arena = arena_create (1028);
    ASSERT_NOT_NULL (arena);

    Array* array = array_create (arena, 10, sizeof (int), alignof (int));
    ASSERT_NOT_NULL (array);

    /*
     * Get the first element.
     */
    int value = 10;
    int* first = (int*)array_set (array, 0, &value);
    ASSERT_NOT_NULL (first);
    EXPECT_EQ (*(int*)array_get (array, 0), value);

    /*
     * Get a different element.
     */
    value = 3;
    int* fourth = (int*)array_set (array, 3, &value);
    ASSERT_NOT_NULL (fourth);
    EXPECT_EQ (*(int*)array_get (array, 3), value);

    /*
     * Get the last element.
     */
    value = 9;
    int* last = (int*)array_set (array, -1, &value);
    ASSERT_NOT_NULL (last);
    EXPECT_EQ (*(int*)array_get (array, -1), value);

    /*
     * Exceed the bounds.
     */
    int* bad = (int*)array_set (array, 10, &value);
    EXPECT_NULL (bad);

    /*
     * Exceed the bounds in the other way
     */
    int* negative_bad = (int*)array_set (array, -11, &value);
    EXPECT_NULL (negative_bad);

    arena_free (arena);
}
