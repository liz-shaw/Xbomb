# 贪心算法

## 适用贪心算法的问题

### 贪心选择性质（greedy-choice property)

**定义**：我们可以通过做出**局部最优**的选择，从而最终导致**全局最优解**。

**CLRS的观点**：在动态规划中，我们通常需要先解决子问题，才能做出当前的选择；而在贪心算法中，我们可以在不考虑子问题结果的情况下，直接做出当前最好的选择，然后再去解决剩下的唯一子问题。

**证明方法**：书中强调，要证明一个问题具有贪心选择性质，通常需要使用**替换论证法**（Exchange Argument）：假设存在一个最优解但不包含你的贪心选择，通过证明将其中的元素替换为贪心选择后，解的质量不会变差。

###  最优子结构（Optimal Substructure)

**定义**：一个问题的最优解包含其子问题的最优解。

**在贪心中的应用**：如果你做出了贪心选择，原问题就简化为一个规模更小的子问题。只要原问题具有最优子结构，那么“贪心选择 + 子问题的最优解”就能合成原问题的最优解。



## 贪心算法设计的通用步骤

1. **转化问题：** 将原问题设定为：做一个选择后，只剩下一个子问题。
2. **证明安全：** 证明在原始问题中做一个贪心选择总是“安全的”（即总能得到一个**包含该选择**的最优解）。
3. **证明子结构：** 证明做出选择后，剩余的子问题加上你的贪心选择，确实构成了原问题的最优解。

## 其他用在贪心算法中的技巧

- 先排序，后处理

- 双指针扫描

  

# leetcode

### 455

```c++
#include <vector>
#include <algorithm>
#include <iostream>

using namespace std;

// 全局静态优化：提升评测机执行效率
static const int _ = []() {
    ios::sync_with_stdio(false); // 优化输入输出流
    cin.tie(nullptr);
    return 0;
}();

class Solution {
public:
    int findContentChildren(vector<int>& g, vector<int>& s) {
        // 1. 预处理：利用快排实现单调递增
        sort(g.begin(), g.end());
        sort(s.begin(), s.end());

        int child_idx = 0;  // 需求端指针（同时作为满足计数值）
        int cookie_idx = 0; // 资源端指针

        const int n = g.size();
        const int m = s.size();

        // 2. 线性扫描：利用贪心性质进行匹配
        while (child_idx < n && cookie_idx < m) {
            // 贪心选择：当前最小资源是否满足当前最小需求
            if (s[cookie_idx] >= g[child_idx]) {
                child_idx++; // 满足需求，移动需求指针
            }
            cookie_idx++; // 无论是否满足，当前最小资源已处理（单向性）
        }
        
        return child_idx; // 返回满足的孩子总数
    }
};
```

