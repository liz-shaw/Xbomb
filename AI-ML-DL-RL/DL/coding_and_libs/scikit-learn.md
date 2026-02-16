# Scikit-learn

## transformation（数据变换流程）

### 顺序（Pipeline 顺序约束）

#### 特征扩展

- 多项式
- Polynomial Transformer（PolynomialFeatures）

------

#### 归一化 / 标准化

- MinMaxScaler
- StandardScaler

##### 流程

- fit()
- transform()

##### 注意事项

- 验证集不能用于 fit
- 测试集不能用于 fit
- 只能使用训练集 fit
- 验证 / 测试只做 transform

------

#### 特征提取

- PCA

------

## estimator（估计器模型接口）

- fit()
- predict()
- score()

------

## ensemble（集成学习器）

- Bagging
- Random Forest
- Boosting 系列

------

