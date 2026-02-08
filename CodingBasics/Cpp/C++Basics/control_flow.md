# 跳转语句

## if-else

```c++
include <iostream>

int main() {
    int age = 18;

    if (age >= 18) {
        std::cout << "You can vote!" << std::endl;
    } else {
        std::cout << "Too young to vote." << std::endl;
    }

    return 0;
}
```



## switch

```c++
#include <iostream>

int main() {
    int day = 3;

    switch (day) {
        case 1: std::cout << "Monday"; break;
        case 2: std::cout << "Tuesday"; break;
        case 3: std::cout << "Wednesday"; break;
        default: std::cout << "Invalid day";
    }

    return 0;
}
```



## goto

```c++
#include <iostream>

int main() {
    int x = 5;

    if (x == 5)
        goto label;

    std::cout << "This line will be skipped." << std::endl;

label:
    std::cout << "Jumped to label!" << std::endl;

    return 0;
}
```





# 循环体

## For Loop

```c++
for (initialization; condition; increment/decrement) {
    // block of code to execute
}
```

## While Loop

```c++
while (condition) {
    // block of code to execute
}
```





## Do-While Loop

```
do {
    // block of code to execute
} while (condition);
```