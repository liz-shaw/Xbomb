# C++ 中的数据类型（Data Types in C++）

在 C++ 中，数据类型用于对程序可以处理的不同类型的数据进行分类。
 它们决定了变量可以存储的值的类型，以及变量在内存中占用的空间大小。
 C++ 中的一些基本数据类型包括：整数、浮点数、字符和布尔值。

------

## 基本数据类型（Fundamental Data Types）

### 整型（int）

整型用于表示整数，可以存储正数和负数。
 `int` 的大小取决于系统架构（通常为 4 字节）。

示例：

```c++
int num = 42;
```

`int` 还有一些变体，用于表示不同范围的整数：

- `short`（short int）：比 int 范围小
- `long`（long int）：比 int 范围大
- `long long`（long long int）：比 long 范围更大

------

### 浮点型（float, double）

浮点型用于表示实数，即带小数点的数。

- `float`：单精度浮点数，通常占用 4 字节内存

示例：

```c++
float pi = 3.14f;
```

- `double`：双精度浮点数，通常占用 8 字节内存，精度高于 float

示例：

```c++
double pi_high_precision = 3.1415926535;
```

------

### 字符型（char）

字符型用于表示单个字符，如字母、数字或符号。
 通常使用 ASCII 编码，占用 1 字节内存。

示例：

```
char letter = 'A';
```

------

### 布尔型（bool）

布尔型表示逻辑值：`true` 或 `false`。
 通常占用 1 字节内存。

示例：

```c++
bool is_cpp_great = true;
```

------

## 派生数据类型（Derived Data Types）

派生数据类型是由基本数据类型派生而来，例如：

------

### 数组（Arrays）

数组用于在连续的内存位置中存储多个相同类型的值。

示例：

```c++
int numbers[5] = {1, 2, 3, 4, 5};
int scores[10] = {100, 95, 98}; // 前 3 个元素初始化，其余初始化为 0
int allZero[0] = {0};           // 全部初始化为 0
```

------

### 指针（Pointers）

指针用于存储变量的内存地址。

示例：

```c++
int num = 42;
int* pNum = &num;
```

------

### 引用（References）

引用是共享内存位置的一种方式，相当于为变量创建一个别名。

示例：

```c++
int num = 42;
int& numRef = num;
```

------

## 用户自定义数据类型（User-Defined Data Types）

用户自定义数据类型由程序员定义，例如结构体、类和联合体。

------

### 结构体（struct）

结构体用于在一个变量中存储不同类型的数据，
 其成员变量和成员函数默认是 `public`。

示例：

```c++
struct Person {
    std::string name;
    int age;
    float height;
};

Person p1 = {"John Doe", 30, 5.9};
```

------

### 类（class）

类与结构体类似，但成员的访问权限由访问控制符决定，
 默认情况下成员是 `private`。

示例：

```c++
class Person {
public:
    std::string name;
    int age;

    void printInfo() {
        std::cout << "Name: " << name << ", Age: " << age << '\n';
    };
};

Person p1;
p1.name = "John Doe";
p1.age = 30;
```

------

### 联合体（union）

联合体用于在同一块内存中存储不同的数据类型。

示例：

```c++
union Data {
    int num;
    char letter;
    float decimal;
};

Data myData;
myData.num = 42;
```



# 静态类型和动态类型

## 静态类型（Static Typing）

在 C++ 中，**变量类型在编译期确定**，运行时不能改变。

- 类型检查发生在 **编译期**
- 类型不匹配 → 编译错误或编译期转换

示例：

```
int a = 10;
double b = 3.14;

a = b;   // double → int，截断
```

------

## 动态类型（Dynamic Typing in C++）

C++ **不是动态类型语言**，只提供有限的动态能力。

------

### void*

```c++
void* p;
int x = 10;

p = &x;
int y = *(int*)p;
```

- 不带类型信息
- 需要手动强转

------

### std::any（C++17）

```c++
#include <any>

std::any v;
v = 10;
v = 3.14;
```

- 运行时保存类型
- 取值需 `std::any_cast<T>`

------

## 对比速记

| 项       | 静态类型 | 动态类型 |
| -------- | -------- | -------- |
| 类型确定 | 编译期   | 运行期   |
| C++ 常用 | 是       | 否       |