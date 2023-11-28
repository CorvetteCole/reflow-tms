import numpy as np
import numpy.linalg as npl
import scipy.linalg as spl
import scipy.optimize as spo
from tqdm import tqdm


class LinearMPCFactor:
    decPlace = 6

    def __init__(self, A: np.ndarray, B: np.ndarray, Q: np.ndarray, R: np.ndarray, N: int,
                 A_x: np.ndarray, b_x: np.ndarray, A_u: np.ndarray, b_u: np.ndarray):
        self.A = A
        self.B = B
        self.Q = Q
        self.R = R
        self.QN, self.K = self.TermPenalMatCal()

        self.A_x = A_x
        self.b_x = b_x
        self.A_u = A_u
        self.b_u = b_u

        self.N = N

        A_d = np.vstack((A_x, A_u @ self.K))
        b_d = np.hstack((b_x, b_u))
        self.FN, self.cN = self.TermSetCal(A_d, b_d)

        self.F = np.vstack((A_x, np.zeros((A_u.shape[0], A.shape[1]))))
        self.G = np.vstack((np.zeros((A_x.shape[0], B.shape[1])), A_u))
        self.c = np.hstack((b_x, b_u))

        self.L_phi = self.LipConstCal()

    # 通过离散lqr计算出终端惩罚矩阵P和用来镇定实际状态和名义系统状态之差的矩阵K
    def TermPenalMatCal(self):
        p = spl.solve_discrete_are(self.A, self.B, self.Q, self.R)  # 求Riccati方程
        k = npl.inv(self.R + self.B.T @ p @ self.B) @ self.B.T @ p @ self.A

        return p, k

    # 计算终端约束区域Xf
    def TermSetCal(self, Ad: np.ndarray, bd: np.ndarray):
        t = 0
        ACons, bCons = Ad, bd  # 初始状态
        Ak = self.A - self.B @ self.K
        progress = tqdm(desc="Calculating terminal set")

        while True:
            max_res = []

            for i in range(0, Ad.shape[0]):
                c = Ad[i, :] @ np.linalg.matrix_power(Ak, t + 1)
                bounds = [(None, None)] * Ad.shape[1]
                res = spo.linprog(-c, A_ub=ACons, b_ub=bCons, bounds=bounds, method='revised simplex')
                max_res.append(-res.fun)
            # 检验Ot+1是否与Ot相等

            if ((np.array(max_res) - bd) <= 0).all():
                break  # 若相等则跳出循环
            else:
                progress.write(f"t = {t}, max_res - bd = {np.array(max_res) - bd}")

            t = t + 1

            ACons = np.vstack((ACons, Ad @ npl.matrix_power(Ak, t)))
            bCons = np.hstack((bCons, bd))
            # 若不是则令Ot = Ot+1继续循环
            progress.update()

        ACons, bCons = self.rmCollinear(ACons, bCons)
        ACons, bCons = self.rmRedundant(ACons, bCons)  # 得到结果后还需除去冗余项
        # 计算方法是增加t，直到Ot == Ot+1，于是有O∞ = Ot
        return ACons, bCons

    # 去除一个线性不等式矩阵(A,b)中共线的边
    @staticmethod
    def rmCollinear(a: np.ndarray, b: np.ndarray):
        c = np.hstack((a, b.reshape(-1, 1)))
        delete_line = []

        for i in range(0, c.shape[0]):
            for j in range(i + 1, c.shape[0]):
                test_arr = np.vstack((c[i, :], c[j, :]))
                if npl.matrix_rank(test_arr) < 2:
                    delete_line.append(j)

        c = np.delete(c, delete_line, 0)

        new_a = np.delete(c, -1, 1)
        new_b = c[:, -1]

        return new_a, new_b

    # 去除冗余项
    @staticmethod
    def rmRedundant(a: np.ndarray, b: np.ndarray):
        delete_line = []
        for i in range(0, a.shape[0]):
            c = a[i, :]
            a_bar = np.delete(a, i, 0)
            b_bar = np.delete(b, i)
            bounds = [(None, None)] * c.shape[0]
            res = spo.linprog(-c, A_ub=a, b_ub=b, bounds=bounds, method='revised simplex')
            res_bar = spo.linprog(-c, A_ub=a_bar, b_ub=b_bar, bounds=bounds, method='revised simplex')

            if 0.0001 > res.fun - res_bar.fun > -0.0001:
                delete_line.append(i)
            # 有它没它都一样的话，说明该项冗余

        new_a = np.delete(a, delete_line, 0)
        new_b = np.delete(b, delete_line)

        return new_a, new_b

    # 求李普希茨常数
    def LipConstCal(self):
        M = np.zeros((self.N * (self.Q.shape[0] + self.R.shape[0]) + self.QN.shape[0],
                      self.N * (self.Q.shape[1] + self.R.shape[1]) + self.QN.shape[1]))

        for i in range(self.N):
            M[self.Q.shape[0] * i:self.Q.shape[0] * (i + 1),
            self.Q.shape[1] * i:self.Q.shape[1] * (i + 1)] = self.Q
            M[self.Q.shape[0] * self.N + self.QN.shape[0] + self.R.shape[0] * i:
              self.Q.shape[0] * self.N + self.QN.shape[0] + self.R.shape[0] * (i + 1),
            self.Q.shape[1] * self.N + self.QN.shape[1] + self.R.shape[1] * i:
            self.Q.shape[1] * self.N + self.QN.shape[1] + self.R.shape[1] * (i + 1)] = self.R
            M[self.Q.shape[0] * self.N + self.QN.shape[0] + self.R.shape[0] * i:
              self.Q.shape[0] * self.N + self.QN.shape[0] + self.R.shape[0] * (i + 1),
            self.Q.shape[1] * i:
            self.Q.shape[1] * (i + 1)] = np.zeros((self.R.shape[0], self.Q.shape[1]))
            M[self.Q.shape[0] * i:
              self.Q.shape[0] * (i + 1),
            self.Q.shape[1] * self.N + self.QN.shape[1] + self.R.shape[1] * i:
            self.Q.shape[1] * self.N + self.QN.shape[1] + self.R.shape[1] * (i + 1)] = np.zeros((self.R.shape[0], self.Q.shape[1])).T

        M[self.Q.shape[0] * self.N:self.Q.shape[0] * self.N + self.QN.shape[0],
        self.Q.shape[1] * self.N:self.Q.shape[1] * self.N + self.QN.shape[1]] = self.QN

        g = np.zeros((self.F.shape[0] * self.N + self.FN.shape[0],
                      (self.F.shape[1] + self.G.shape[1]) * self.N + self.FN.shape[1]))

        for i in range(self.N):
            g[self.F.shape[0] * i:self.F.shape[0] * (i + 1), self.F.shape[1] * i:self.F.shape[1] * (i + 1)] = self.F
            g[self.G.shape[0] * i:
              self.G.shape[0] * (i + 1),
            self.F.shape[1] * self.N + self.FN.shape[1] + self.G.shape[1] * i:
            self.F.shape[1] * self.N + self.FN.shape[1] + self.G.shape[1] * (i + 1)] = self.G

        g[self.F.shape[0] * self.N:self.F.shape[0] * self.N + self.FN.shape[0],
        self.F.shape[1] * self.N:self.F.shape[1] * self.N + self.FN.shape[1]] = self.FN

        LipConst = ((npl.norm(g, 2)) ** 2) / np.min(npl.eigvals(M))
        # print(np.eigvals(M))  # 如果Q或R中有较小的值，建议上式分母为一个尽可能小但不接近零的特征值而不是最小特征值
        return LipConst

    # 打印初始化所需数组的cpp代码
    def PrintCppArray(self, mat: np.ndarray, name: str):
        n = mat.size
        lst = mat.reshape(-1)
        print(f"MatDataType_t " + name + f"_arr[{n}] = " + "{", end="")
        for i in range(n - 1):
            print(f"{np.around(lst[i], self.decPlace)}, ", end="")
        print(np.around(lst[-1], self.decPlace), end="")
        print("};")

    # 打印初始化所需矩阵的cpp代码
    @staticmethod
    def PrintCppMatrix(mat: np.ndarray, name: str):
        if len(mat.shape) == 1:
            row, = mat.shape
            column = 1
        else:
            row, column = mat.shape
        print(f"Matrix " + name + f" = Matrix({row}, {column}, " + name + "_arr);")

    # 打印求解器初始化cpp代码
    def PrintCppCode(self, epsilon_V: float, epsilon_g: float, max_iter: int):
        print(f"MatDataType_t L_phi = {np.around(self.L_phi, self.decPlace)};")
        print(f"MatDataType_t e_V = {epsilon_V};")
        print(f"MatDataType_t e_g = {epsilon_g};")
        print(f"uint32_t max_iter = {max_iter};")
        print(f"uint32_t N = {self.N};")

        print()

        self.PrintCppArray(self.A, "A")
        self.PrintCppArray(self.B, "B")
        self.PrintCppArray(self.Q, "Q")
        self.PrintCppArray(self.R, "R")
        self.PrintCppArray(self.QN, "QN")
        self.PrintCppArray(self.F, "F")
        self.PrintCppArray(self.G, "G")
        self.PrintCppArray(self.c, "c")
        self.PrintCppArray(self.FN, "FN")
        self.PrintCppArray(self.cN, "cN")

        print()

        self.PrintCppMatrix(self.A, "A")
        self.PrintCppMatrix(self.B, "B")
        self.PrintCppMatrix(self.Q, "Q")
        self.PrintCppMatrix(self.R, "R")
        self.PrintCppMatrix(self.QN, "QN")
        self.PrintCppMatrix(self.F, "F")
        self.PrintCppMatrix(self.G, "G")
        self.PrintCppMatrix(self.c, "c")
        self.PrintCppMatrix(self.FN, "FN")
        self.PrintCppMatrix(self.cN, "cN")

        print()

        print(f"MPCController mpc = MPCController(L_phi, e_V, e_g, max_iter, N, A, B, Q, R, QN, F, G, c, FN, cN);")
