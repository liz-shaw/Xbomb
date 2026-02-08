# C++

## C++ 里的“栈”怎么用

这里分三层：

1. **函数调用栈（call stack）**：自动管理，你平时用局部变量就是在用它。
2. **标准库栈容器 `std::stack`**：LIFO 数据结构封装。
3. **用 `vector`/`string` 自己当栈**：刷题、工程里非常常用，比 `std::stack` 更灵活

## 1. 函数调用栈（理解为主）

```c++
void f() {
    int x = 10;     // x 在栈上分配
}                   // 离开作用域，x 自动销毁
```

特点：

- 作用域结束自动释放，**不需要也不能 `delete`**。
- 适合小而固定大小的对象。
- 递归太深会栈溢出（stack overflow）。

这是语言层面的，不是你手动“用栈容器”。

## 2. `std::stack`：标准栈容器

头文件 & 定义：

```c++
#include <stack>

std::stack<int> st;
```

常用操作（核心就 5 个）：

```c++
st.push(10);      // 压栈
st.push(20);

st.top();         // 访问栈顶：20
st.pop();         // 弹出栈顶（不返回值）

st.empty();       // 是否为空
st.size();        // 元素个数
```

**规范用法示例：括号匹配**

```c++
#include <bits/stdc++.h>  // 比赛可用，工程中建议按需包含头文件
using namespace std;

bool isValid(const string& s) {
    stack<char> st;
    for (char c : s) {
        if (c == '(') {
            st.push(c);
        } else if (c == ')') {
            if (st.empty()) return false;
            st.pop();
        }
    }
    return st.empty();
}

int main() {
    cout << isValid("(())") << endl;  // 1
    cout << isValid("(()") << endl;   // 0
}
```

**注意点（工程风格）**：

- 头文件推荐：`#include <stack>`，不要全用 `<bits/stdc++.h>`。

- 不要对空栈调 `top()` / `pop()`，行为未定义（UB）。

- `std::stack` 默认基于 `deque`，如果要基于 `vector`：

  ```c++
  std::stack<int, std::vector<int>> st2;
  ```

------

## 3. 用 `vector` 当栈（实战中更常见）

竞赛 / 工程中很多人更喜欢：

```c++
#include <vector>

std::vector<int> st;
st.push_back(10);          // push
int x = st.back();         // top
st.pop_back();             // pop
bool empty = st.empty();
```

优点：可遍历、可随机访问，比 `std::stack` 更灵活。

### 1. 为什么用 `vector` 做栈？

`std::stack` 是一个**适配器**，默认内部就是用 `std::deque`，接口只给你：

- `push`
- `pop`
- `top`
- `empty`
- `size`

很安全，但很憋屈：不能遍历、不能随机访问、调试也不方便。

而 `std::vector`：

- 内存连续，cache 友好；
- 尾部操作：`push_back`、`pop_back`、`back` 摊还 O(1)；
- 还能遍历、索引、调试，刷题 & 工程实战都很好用。

所以大家说「vector 栈」= **“用 vector 实现 LIFO 栈”**。

------

### 2. C++：`vector` 当栈的规范用法

核心约定：**只在末尾操作**。

```c++
#include <vector>
#include <iostream>
using namespace std;

int main() {
    vector<int> st;      // 把它当栈用

    // 压栈
    st.push_back(10);
    st.push_back(20);
    st.push_back(30);

    // 访问栈顶
    int top1 = st.back();    // 30

    // 弹栈
    st.pop_back();           // 删除 30

    int top2 = st.back();    // 20

    // 判空
    bool empty = st.empty(); // false

    // 遍历（std::stack 做不到）
    for (int x : st) {
        cout << x << " ";    // 10 20
    }
}
```

**风格和注意点：**

1. 当栈用时，不要在中间插入/删除（`insert/erase`），破坏“栈”的语义，复杂度也高。
2. `back()` / `pop_back()` 前务必确保 `!empty()`，否则未定义行为（UB）。
3. 适用于：单调栈、DFS 手写栈、中缀转后缀、括号匹配、区间合并等。

------

### 3. 用 `vector` 写一个小栈封装（更清晰）

这样能强化你对“抽象数据结构”的意识：

```c++
#include <vector>
using namespace std;

template<typename T>
class VecStack {
private:
    vector<T> data;
public:
    void push(const T& x) { data.push_back(x); }
    void pop() { data.pop_back(); }
    T& top() { return data.back(); }
    bool empty() const { return data.empty(); }
    size_t size() const { return data.size(); }
};
```

本质：直接把 `vector` 的尾部接口暴露出来，就是一个干净的栈。

# Python 的“栈”

Python 没有专门的 `stack` 类型，一般用：

1. `list`：默认首选。
2. `collections.deque`：更适合频繁两端操作。
3. `queue.LifoQueue`：多线程场景。

## 1. `list` 作为栈（最常用）

```python
stack = []

stack.append(10)   # push
stack.append(20)

top = stack[-1]    # 访问栈顶，不弹出 -> 20
x = stack.pop()    # 弹出 -> 20

empty = (len(stack) == 0)
```

示例：同样是括号匹配

```
def is_valid(s: str) -> bool:
    stack = []
    for c in s:
        if c == '(':
            stack.append(c)
        elif c == ')':
            if not stack:
                return False
            stack.pop()
    return not stack

print(is_valid("(())"))  # True
print(is_valid("(()"))   # False
```

## 2. `deque` 作为栈（更稳一点）

```python
from collections import deque

stack = deque()
stack.append(10)
stack.append(20)
top = stack[-1]
x = stack.pop()
```

说明：

- 对于只在尾部 `append` / `pop` 的场景，`list` 已经是均摊 O(1)，足够好。
- `deque` 更适合双端操作（既做栈又做队列）。

## 3. `LifoQueue`（需要线程安全时）

```python
from queue import LifoQueue

stack = LifoQueue()
stack.put(10)
stack.put(20)
top = stack.get()   # 阻塞式弹出
```