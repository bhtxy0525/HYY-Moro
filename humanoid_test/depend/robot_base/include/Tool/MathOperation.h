/**
 * @file MathOperation.h
 *
 * @brief  数学运算
 * @author hanbing
 * @version 13.2.2
 * @date 2026-07-25
 *
 */

#ifndef MATHOPERATION_H_
#define MATHOPERATION_H_



#ifdef __cplusplus
namespace HYYRobotBase
{
extern "C" {
#endif

/**
 * @brief 三维向量叉乘运算
 * @param[in]  x1  第一个输入三维向量
 * @param[in]  x2  第二个输入三维向量
 * @param[out] y   叉乘结果输出向量，满足 y = x1 × x2
 */
extern void MultCross(double x1[3], double x2[3], double y[3]);

/**
 * @brief 3×3旋转矩阵相乘运算
 * @param[in]  R1   第一个输入 3×3 旋转矩阵
 * @param[in]  R2   第二个输入 3×3 旋转矩阵
 * @param[out] Rres 矩阵相乘结果，满足 Rres = R1 * R2
 */
extern void RMultR(double(*R1)[3], double(*R2)[3], double(*Rres)[3]);

/**
 * @brief 3×3旋转矩阵左乘三维向量
 * @param[in]  R     输入 3×3 旋转矩阵
 * @param[in]  v     输入三维向量
 * @param[out] vres  矩阵向量乘积，满足 vres = R * v
 */
extern void RMultVec(double(*R)[3], double v[3], double vres[3]);

/**
 * @brief 计算两个三维向量的点积（内积）
 * @param[in] v1  第一个输入三维向量
 * @param[in] v2  第二个输入三维向量
 * @return 两向量的点积标量结果
 */
extern double VecMultVec(double v1[3], double v2[3]);

/**
 * @brief 三维向量加法运算
 * @param[in]  v1   第一个输入三维向量
 * @param[in]  v2   第二个输入三维向量
 * @param[out] res  向量相加结果，满足 res = v1 + v2
 */
extern void VecAddVec(double v1[3], double v2[3], double res[3]);

/**
 * @brief 三维向量减法运算
 * @param[in]  v1   被减三维向量
 * @param[in]  v2   减三维向量
 * @param[out] res  向量相减结果，满足 res = v1 - v2
 */
extern void VecSubVec(double v1[3], double v2[3], double res[3]);

/**
 * @brief 三维向量数乘（标量乘法）运算
 * @param[in]  v1     输入三维向量
 * @param[in]  scale  标量乘数
 * @param[out] res    数乘结果，满足 res = v1 * scale
 */
extern void VecDot(double v1[3], double scale, double res[3]);

/**
 * @brief 计算三维向量的模长（欧几里得二范数）
 * @param[in] v1  输入三维向量
 * @return 向量的模长值
 */
extern double VecModule(double v1[3]);

/**
 * @brief 三维向量单位化（归一化）
 * @param[in]  v1   输入三维向量
 * @param[out] res  单位化后的输出向量
 * @note 若输入向量模长<1e-10，输出零向量
 */
extern void VecUnit(double v1[3], double res[3]);

/**
 * @brief 计算两个三维向量之间的夹角
 * @param[in] v1  第一个输入三维向量
 * @param[in] v2  第二个输入三维向量
 * @return 两向量的夹角值，单位为弧度
 */
extern double VecAangleOff(double v1[3], double v2[3]);

/**
 * @brief 3×3旋转矩阵原地转置
 * @param[in,out] R  待转置的 3×3 旋转矩阵，转置结果直接覆盖原矩阵
 */
extern void RTransposition(double(*R)[3]);

/**
 * @brief 计算 3×3 旋转矩阵的转置并输出到新矩阵
 * @param[in]  R     输入 3×3 旋转矩阵（原矩阵保持不变）
 * @param[out] Rres  转置后的输出矩阵
 */
extern void RTranspositionR(double(*R)[3], double(*Rres)[3]);

/**
 * @brief 4×4 齐次变换矩阵原地求逆
 * @param[in,out] T  待求逆的 4×4 齐次变换矩阵，求逆结果直接覆盖原矩阵
 * @note 仅适用于标准刚体变换矩阵（旋转正交、最后一行为 [0 0 0 1]）
 */
extern void Tinverse(double(*T)[4]);

/**
 * @brief 计算 4×4 齐次变换矩阵的逆并输出到新矩阵
 * @param[in]  T     输入 4×4 齐次变换矩阵（原矩阵保持不变）
 * @param[out] Tres  求逆后的输出变换矩阵
 * @note 仅适用于标准刚体变换矩阵（旋转正交、最后一行为 [0 0 0 1]）
 */
extern void TinverseT(double(*T)[4], double(*Tres)[4]);

/**
 * @brief 4×4 齐次变换矩阵相乘运算
 * @param[in]  T1    第一个输入 4×4 变换矩阵
 * @param[in]  T2    第二个输入 4×4 变换矩阵
 * @param[out] Tres  矩阵相乘结果，满足 Tres = T1 * T2
 */
extern void TMultT(double(*T1)[4], double(*T2)[4], double(*Tres)[4]);

/**
 * @brief 4×4 齐次变换矩阵左乘三维向量
 * @param[in]  T     输入 4×4 齐次变换矩阵
 * @param[in]  v     输入三维笛卡尔向量（自动补齐次分量 1）
 * @param[out] vres  变换后的三维输出向量
 */
extern void TMultVec(double(*T)[4], double v[3], double vres[3]);

/**
 * @brief 将 3×3 旋转矩阵转换为 4×4 齐次变换矩阵
 * @param[in]  R  输入 3×3 旋转矩阵
 * @param[out] T  输出 4×4 齐次变换矩阵，平移部分初始化为零向量
 */
extern void r2t(double(*R)[3], double(*T)[4]);

/**
 * @brief 从 4×4 齐次变换矩阵中提取 3×3 旋转矩阵
 * @param[in]  T  输入 4×4 齐次变换矩阵
 * @param[out] R  提取出的 3×3 旋转矩阵
 */
extern void t2r(double(*T)[4], double(*R)[3]);

/**
 * @brief 设置 4×4 齐次变换矩阵的平移分量
 * @param[in,out] T  待设置平移的 4×4 齐次变换矩阵
 * @param[in]     P  输入三维平移向量
 */
extern void transl(double(*T)[4], double P[3]);

/**
 * @brief 计算两个 4×4 齐次变换矩阵的位姿偏差
 * @param[in]  T1    参考位姿变换矩阵
 * @param[in]  T2    目标位姿变换矩阵
 * @param[out] resv  输出 6 维偏差向量，前 3 维为平移误差，后 3 维为姿态误差
 */
extern void T2delta(double(*T1)[4], double(*T2)[4], double resv[6]);

/**
 * @brief 通用二维矩阵转置运算
 * @param[in]  M     输入二维矩阵
 * @param[in]  row   输入矩阵的行数
 * @param[in]  col   输入矩阵的列数
 * @param[out] Mres  转置后的输出矩阵，尺寸为 col 行 row 列
 */
extern void MatrixTrans(double** M, int row, int col, double** Mres);

/**
 * @brief 计算 n 阶方阵的行列式值
 * @param[in] M  输入 n 阶方阵
 * @param[in] n  矩阵的阶数
 * @return 矩阵的行列式计算结果
 */
extern double MatrixDeterminant(double** M,int n);

/**
 * @brief 计算 6×6 矩阵的行列式值
 * @param[in] matrix  输入 6×6 方阵
 * @return 6×6 矩阵的行列式计算结果
 */
extern double MatrixDeterminant6x6(double matrix[6][6]);

/**
 * @brief n 阶方阵求逆运算
 * @param[in]  M      输入 n 阶可逆方阵
 * @param[out] M_res  求逆后的输出矩阵
 * @param[in]  n      矩阵的阶数
 * @return 求逆后的矩阵指针（即 M_res），若矩阵不可逆则返回 NULL
 */
extern double** MatrixInverse(double** M, double** M_res, int n);

/**
 * @brief 创建二维指针
 * @param name 指针名
 */
#define CREATEDOUBLEPOINTER(name)  \
        double _____##name##__[ROBOT_MAX_DOF][ROBOT_MAX_DOF]; \
        double *______##name##__[ROBOT_MAX_DOF]; \
        double **name =______##name##__; \
		do{\
			int i=0; \
            for (i=0;i<ROBOT_MAX_DOF;i++) \
            { \
                ______##name##__[i]=_____##name##__[i]; \
            } \
		}while(0)


#ifdef __cplusplus
}
}
#endif


#endif /* MATHOPERATION_H_ */
