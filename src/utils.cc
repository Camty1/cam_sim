/**
 * @file utils.cc Contains various utilities used in camsim.
 */
#include "utils.h"
#include <cstring>

Arena*
arena_create (size_t capacity)
{
    /*
     * Allocate memory for buffer and arena.
     */
    char* buffer = (char*)calloc (capacity, sizeof (char));
    if (buffer == NULL)
    {
        return NULL;
    }
    Arena* arena = (Arena*)calloc (1, sizeof (Arena));
    if (arena == NULL)
    {
        return NULL;
    }

    /*
     * Set parameters of arena.
     */
    arena->buffer = buffer;
    arena->capacity = capacity;

    return arena;
}

void
arena_free (Arena* arena)
{
    free (arena->buffer);
    free (arena);
}

void*
arena_allocate (Arena* arena, size_t size, size_t alignment)
{
    /*
     * Determine how much padding we need to maintain alignment.
     */
    size_t padding = 0;
    size_t mod = (arena->offset % alignment);
    if (mod != 0)
    {
        padding = alignment - mod;
    }

    /*
     * Check if arena has space for object (considering alignment).
     */
    if (arena->offset + padding + size > arena->capacity)
    {
        return NULL;
    }

    /*
     * Align offset.
     */
    arena->offset += padding;

    /*
     * Set pointer
     */
    void* pointer = arena->buffer + arena->offset;

    /*
     * Update offset to end of object.
     */
    arena->offset += size;

    return pointer;
}

void*
arena_multi_allocate (Arena* arena, size_t number, size_t size, size_t alignment)
{
    return arena_allocate (arena, number * size, alignment);
}

/*
 * A private method used to create an empty string (as strings are treated as immutable when using
 * the interface).
 *
 * @param arena The arena used for memory allocation.
 * @param size The size of the string.
 *
 * @return A pointer to the new empty string, or NULL if allocation failed.
 */
String*
string_create_empty (Arena* arena, size_t size)
{
    /*
     * Check the size of the string.
     */
    if (size > MAX_STRING_SIZE)
    {
        return NULL;
    }

    /*
     * Create the string object, and check that allocation happened correctly.
     */
    String* string = arena_allocate_type (arena, String);
    if (string == NULL)
    {
        return NULL;
    }
    string->size = size;
    string->text = arena_multi_allocate_type (arena, size, char);
    if (string->text == NULL)
    {
        return NULL;
    }

    return string;
}

const String*
string_create (Arena* arena, const char* text)
{
    /*
     * Get the size of the string by looking for a null terminator.  Fail if the size is greater
     * than MAX_STRING_SIZE.
     */
    int size = 0;
    while (text[size] != '\0' and size <= MAX_STRING_SIZE)
    {
        size++;
    }
    if (size > MAX_STRING_SIZE)
    {
        return NULL;
    }

    /*
     * Create the string object, and check that allocation happened correctly.
     */
    String* string = string_create_empty (arena, size);
    if (string == NULL)
    {
        return NULL;
    }

    /*
     * Copy over the old string into the new string.  Check that memcpy succeeded.
     */
    char* result = (char*)memcpy (string->text, text, size);
    if (result != string->text)
    {
        return NULL;
    }

    return string;
}

char*
string_c_str (Arena* arena, const String* string)
{
    /*
     * Allocate memory for the C string.
     */
    char* c_str = arena_multi_allocate_type (arena, string->size + 1, char);
    if (c_str == NULL)
    {
        return NULL;
    }

    /*
     * Copy over text from String to C string.  Check that memcpy succeeded
     */
    char* result = (char*)memcpy (c_str, string->text, string->size);
    if (result != c_str)
    {
        return NULL;
    }

    /*
     * Add null terminator.
     */
    c_str[string->size] = '\0';

    return c_str;
}

int
string_compare (const String* a, const String* b)
{
    /*
     * Get the size of the shorter string.
     */
    size_t shorter_size = a->size <= b->size ? a->size : b->size;

    /*
     * Compare character by character to find the first different character.
     */
    for (size_t i = 0; i < shorter_size; i++)
    {
        char a_char = a->text[i];
        char b_char = b->text[i];
        if (a_char < b_char)
        {
            return -1;
        }
        if (a_char > b_char)
        {
            return 1;
        }
    }

    /*
     * If we've gotten to this point, the strings match up to the size of the shorter string.  We
     * can now look at how much size is remaining to determine which string is shorter.  If the
     * sizes are the same, then the strings are the same.
     */
    return -(a->size < b->size) + (a->size > b->size);
}

const String*
string_concatenate (Arena* arena, const String* a, const String* b)
{
    /*
     * Create an empty string with the combined size of the two strings.
     */
    size_t size = a->size + b->size;
    String* string = string_create_empty (arena, size);
    if (string == NULL)
    {
        return NULL;
    }

    /*
     * Copy the text from the two strings over.  Check that memcpy succeeded.
     */
    char* result_a = (char*)memcpy (string->text, a->text, a->size);
    if (result_a != string->text)
    {
        return NULL;
    }
    char* result_b = (char*)memcpy (string->text + a->size, b->text, b->size);
    if (result_b != string->text + a->size)
    {
        return NULL;
    }

    return string;
}

const String*
string_file_read (Arena* arena, FILE* file)
{
    /*
     * Check that the file is open.
     */
    if (file == NULL)
    {
        return NULL;
    }

    /*
     * Create an empty string as a buffer.  Make sure allocation works.
     */
    String* string = string_create_empty (arena, MAX_STRING_SIZE);
    if (string == NULL)
    {
        return NULL;
    }
    string->size = 0;

    /*
     * Go character by character until we reach the end of the file.
     */
    int current_char = fgetc (file);
    while (current_char != EOF and string->size < MAX_STRING_SIZE)
    {
        string->text[string->size] = (char)current_char;
        string->size += 1;

        current_char = fgetc (file);
    }

    /*
     * If we ran out of room but didn't get to the end, return NULL
     */
    if (string->size == MAX_STRING_SIZE and current_char != EOF)
    {
        return NULL;
    }

    return string;
}

Array*
array_create (Arena* arena, size_t size, size_t item_size, size_t item_alignment)
{
    /*
     * Allocate the array struct.
     */
    Array* array = arena_allocate_type (arena, Array);
    if (array == NULL)
    {
        return NULL;
    }

    /*
     * Allocate the data buffer for the array.
     */
    array->data = (char*)arena_multi_allocate (arena, size, item_size, item_alignment);
    if (array->data == NULL)
    {
        return NULL;
    }

    /*
     * Populate the rest of the struct
     */
    array->size = size;
    array->item_size = item_size;
    array->item_alignment = item_alignment;

    return array;
}

void*
array_get (Array* array, int index)
{
    if (array == NULL)
    {
        return NULL;
    }
    /*
     * Bounds check.
     */
    if (index < -(int)array->size or index >= (int)array->size)
    {
        return NULL;
    }

    /*
     * Handle negative indices.
     */
    index = index < 0 ? index + (int)array->size : index;

    return (void*)(array->data + (index * array->item_size));
}

void*
array_set (Array* array, int index, void* item)
{
    if (array == NULL)
    {
        return NULL;
    }
    if (item == NULL)
    {
        return NULL;
    }

    /*
     * Get pointer to location in array.
     */
    void* idx_ptr = array_get (array, index);
    if (idx_ptr == NULL)
    {
        return NULL;
    }

    /*
     * Copy item into array.
     *
     * NOTE: We assume here that the pointer to the item we pass is the same as the rest of the
     * elements.  Caveat emptor if you violate this assumption.
     */
    void* result = memcpy (idx_ptr, item, array->item_size);
    /*
     * Check if memcpy failed
     */
    if (result != idx_ptr)
    {
        return NULL;
    }

    return idx_ptr;
}

List*
list_create (Arena* arena, size_t capacity, size_t item_size, size_t item_alignment)
{
    /*
     * Allocate the list struct.
     */
    List* list = arena_allocate_type (arena, List);
    if (list == NULL)
    {
        return NULL;
    }

    /*
     * Allocate the array used by the list.
     */
    list->array = array_create (arena, capacity, item_size, item_alignment);
    if (list->array == NULL)
    {
        return NULL;
    }

    list->size = 0;
    return list;
}

void*
list_get (List* list, int index)
{
    if (list == NULL)
    {
        return NULL;
    }
    /*
     * Bounds check based on size
     */
    if (index < -(int)list->size or index >= (int)list->size)
    {
        return NULL;
    }

    return array_get (list->array, index);
}

void*
list_append (List* list, void* item)
{
    if (list == NULL)
    {
        return NULL;
    }
    if (item == NULL)
    {
        return NULL;
    }

    void* appended_item = array_set (list->array, list->size, item);
    if (appended_item == NULL)
    {
        return NULL;
    }

    list->size++;

    return appended_item;
}

void*
list_set (List* list, int index, void* item)
{
    if (list == NULL)
    {
        return NULL;
    }
    void* item_ptr = list_get (list, index);
    if (item_ptr == NULL)
    {
        return NULL;
    }

    return array_set (list->array, index, item);
}
