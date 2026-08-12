/**
 * @file utils.h
 */
#ifndef CAMSIM_UTILS_H
#define CAMSIM_UTILS_H
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/**
 * Provides a more centralized way to allocate memory.  All allocation calls are
 * made when first allocating the Arena, and then objects will use the memory in
 * the buffer of the arena.  When leaving the scope where the arena is used,
 * simply free the arena and all objects are also freed
 */
typedef struct
{
    /**
     * Memory buffer used for allocation
     */
    char* buffer;

    /**
     * The size of the arena
     */
    size_t capacity;

    /**
     * Beginning of free space in arena
     */
    size_t offset;
} Arena;

/**
 * Performs all allocation to create an arena of the given capacity.
 *
 * @param[in] capacity
 *
 * @return arena Pointer to arena (NULL if buffer or arena alloc fails)
 */
Arena* arena_create (size_t capacity);

/**
 * Frees an arena and the associated memory
 *
 * @param[in] arena
 */
void arena_free (Arena* arena);

/**
 * Allocates memory for an object in the arena.
 *
 * @param[in] arena
 * @param[in] size The size of the object you want to allocate
 * @param[in] alignment The alignment of the object you want to allocate
 *
 * @return pointer The pointer to the object (NULL if arena is full)
 */
void* arena_allocate (Arena* arena, size_t size, size_t alignment);

#define arena_allocate_type(arena, type)                                                           \
    (type*)arena_allocate (arena, sizeof (type), alignof (type))

/**
 * Allocates memory for multiple objects in the arena.
 *
 * @param[in] arena
 * @param[in] number The number of objects you want to allocate
 * @param[in] size The size of the object you want to allocate
 * @param[in] alignment The byte alignment of the object you want to allocate
 *
 * @return pointer The pointer to the first object in the array (NULL if arena is full)
 */
void* arena_multi_allocate (Arena* arena, size_t number, size_t size, size_t alignment);

#define arena_multi_allocate_type(arena, number, type)                                             \
    (type*)arena_multi_allocate (arena, number, sizeof (type), alignof (type))

/**
 * Handles strings (no null terminator).  Strings should be treated as immutable.
 */
typedef struct
{
    char* text;
    size_t size;
} String;

/**
 * The maximum size of a string
 */
#ifdef UNIT_TEST
constexpr int MAX_STRING_SIZE = 1028;
#else
constexpr int MAX_STRING_SIZE = 1028 * 1028;
#endif

/**
 * Creates a string using an arena for memory allocation.
 *
 * @param[in] arena The Arena you want to use for memory allocation.
 * @param[in] text The standard c_str containing the text of the string.
 *
 * @return pointer The pointer to the string or NULL if there is an issue with string creation.
 */
const String* string_create (Arena* arena, const char* text);

/**
 * Converts a string to a C string (adding a null terminator).
 *
 * @param[in] arena The Arena you want to use for memory allocation.
 * @param[in] string The string you want to convert to a C string.
 *
 * @return The string as an array of characters with a null terminator or NULL if there is an issue
 * with allocation.
 */
char* string_c_str (Arena* arena, const String* string);

/**
 * Compares two strings.
 *
 * @param[in] a The first string.
 * @param[in] b The second string.
 *
 * @return -1 if a < b, 0 if a == b, 1 if a > b
 */
int string_compare (const String* a, const String* b);

/**
 * Concatenate two strings.
 *
 * @param[in] arena The arena you want to use for memory allocation.
 * @param[in] a The first string.
 * @param[in] b The second string.
 *
 * @return A pointer to a new string containing the concatenation of a and b, or NULL if
 * concatenation fails.
 */
const String* string_concatenate (Arena* arena, const String* a, const String* b);

/**
 * Read a file into a string.
 *
 * @param[in] arena The arena you want to use for memory allocation.
 * @param[in] file A file pointer, user is responsible for opening and closing the file.
 *
 * @return A pointer to the string containing the file or NULL if reading fails.
 */
const String* string_file_read (Arena* arena, FILE* file);

/**
 * A safe array that enforces bounds checking.
 */
typedef struct
{
    /**
     * The data of the array.
     */
    char* data;

    /**
     * The maximum number of elements in the array.  Do not change after initialization.
     */
    size_t size;

    /**
     * The size (in bytes) of the elements in the array.  Should not change after initialization.
     */
    size_t item_size;

    /**
     * The byte alignment of the elements in the array.  Do not change after initialization
     */
    size_t item_alignment;
} Array;

/**
 * Creates an array of size elements, where each element has a size of item_size and an
 * alignment of item_alignment.
 *
 * @param[in] arena The arena you want to use for memory management.
 * @param[in] size The maximum number of elements in the array.
 * @param[in] item_size The sizeof each element in the array.
 * @param[in] item_alignment The alignof each element in the array.
 *
 * @return A pointer to the array or NULL if allocation fails.
 */
Array* array_create (Arena* arena, size_t size, size_t item_size, size_t item_alignment);

/**
 * Gets an element from an array.  Bounds checking is performed to avoid segfault, but we do not
 * check that you have put anything useful at the index you are accessing.
 *
 * @param[in] array The array to access the element from.
 * @param[in] index The index of the element you wish to access.  A negative index counts backwards
 * from the end of the array, with -1 as the last element (like Python).
 *
 * @return A pointer to the element, or NULL if index is out of bounds.
 */
void* array_get (Array* array, int index);

/**
 * Puts an element into an array.  Bounds checking is performed to avoid segfault.
 *
 * NOTE: We assume the type of the item is the same as the rest of the array.  If you violate this
 * assumption, you take your life into your own hands.  BEWARE.
 *
 * @param[in] array The array to put the item in.
 * @param[in] index The index of the location in the array where you wish to put the element.
 * @param[in] item A pointer to the item you wish to add to the array.
 *
 * @return A pointer to the element in the array, or NULL if bounds checks fail.
 */
void* array_set (Array* array, int index, void* item);

/**
 * A safe list of elements with bounds checking.
 */
typedef struct
{
    /**
     * The array used to store elements in the list
     */
    Array* array;

    /**
     * The current number of elements in the array.
     */
    size_t size;
} List;

/**
 * Creates an empty list with at most capacity elements, where each element has a size of item_size
 * and an alignment of item_alignment.
 *
 * @param[in] arena The arena you want to use for memory management.
 * @param[in] capacity The maximum number of elements in the list.
 * @param[in] item_size The sizeof each element in the list.
 * @param[in] item_alignment The alignof each element in the list.
 *
 * @return A pointer to the list or NULL if allocation fails.
 */
List* list_create (Arena* arena, size_t capacity, size_t item_size, size_t item_alignment);

/**
 * Gets an element from a list.  Bounds checking is performed based on the current number of
 * elements.
 *
 * @param[in] list The list to access the element from.
 * @param[in] index The index of the element you wish to access.  A negative index counts backwards
 * from the end of the list, with -1 as the last element (like Python).
 *
 * @return A pointer to the element, or NULL if index is out of bounds.
 */
void* list_get (List* list, int index);

/**
 * Puts an element at the end of the list.
 *
 * NOTE: We assume the type of the item is the same as the rest of the list.  If you violate this
 * assumption, you take your life into your own hands.  BEWARE.
 *
 * @param[in] list The list to put the item in.
 * @param[in] item A pointer to the item you wish to append to the list.
 *
 * @return A pointer to the element in the list, or NULL if bounds checks fail.
 */
void* list_append (List* list, void* item);

/**
 * Puts an element into an list.  Bounds checking is performed based on the current number of
 * elements.
 *
 * NOTE: We assume the type of the item is the same as the rest of the list.  If you violate this
 * assumption, you take your life into your own hands.  BEWARE.
 *
 * @param[in] list The list to put the item in.
 * @param[in] index The index of the location in the list where you wish to put the element.
 * @param[in] item A pointer to the item you wish to add to the list.
 *
 * @return A pointer to the element in the list, or NULL if bounds checks fail.
 */
void* list_set (List* list, int index, void* item);
#endif /* CAMSIM_UTILS_H */
