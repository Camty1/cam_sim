# EBNF for JSON

```Javascript
Dict = '{' { String ':' Value { ',' String ':' Value } } '}' .
List = '[' { Value { ',' Value } } ']' .
String = '"' { ':' | ',' | '{' | '}' | '[' | ']' | '\n' | NULL | BOOLEAN | DOUBLE | INTEGER } '"' .
Value = Dict | List | String | IDENT | NULL | BOOLEAN | DOUBLE | INTEGER .
```


