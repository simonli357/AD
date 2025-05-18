/**
 * Copyright (c) 2019, Bosch Engineering Center Cluj and BFMC organizers
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:

 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.

 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.

 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.

 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
*/

#ifndef LINALG_H
#define LINALG_H

// #include <mbed.h>
#include <stdint.h>
#include <array>

namespace utils::linalg
{   

    /**
     * @brief CMatrix has aim to implement matrix's functionality. It's a templated class, where the templates defines the type and the size of the matrix. 
     * 
     * @tparam T        variable type of the matrix 
     * @tparam M        number of row in the matrix
     * @tparam N        number of column in the matrix 
     */
    template <class T, uint32_t M, uint32_t N>
    class CMatrix
    {
    public:
        using CThisType = CMatrix<T,M,N>;
        using CTransposeType = CMatrix<T,N,M>;
        using CContainerType = std::array<std::array<T,N>,M>;
        using CDataType =T;

        template <uint32_t P>
        using CLeftMultipliableType = CMatrix<T,P,M>;

        template <uint32_t P>
        using CRightMultipliableType = CMatrix<T,N,P>;

        template <uint32_t P>
        using CLeftMultiplicationResultType = CMatrix<T,P,N>;

        template <uint32_t P>
        using CRightMultiplicationResultType = CMatrix<T,M,P>;

        friend class CMatrix<T,N,M>; 

        CMatrix() : m_data() {}
        CMatrix(const CThisType& f_matrix) : m_data(f_matrix.m_data) {}
        CMatrix(const CThisType&& f_matrix) : m_data(f_matrix.m_data) {}
        CMatrix(const CContainerType& f_data) : m_data(f_data) {}
        CMatrix(const CContainerType&& f_data) : m_data(f_data) {}

        CThisType& operator=(const CThisType& f_matrix)
        {
            for (uint32_t l_row = 0; l_row < M; ++l_row)
            {
                for (uint32_t l_col = 0; l_col < N; ++l_col)
                {
                    this->m_data[l_row][l_col] = f_matrix.m_data[l_row][l_col];
                }
            }
            return *this;
        }
        CThisType& operator=(const CThisType&& f_matrix)
        {
            for (uint32_t l_row = 0; l_row < M; ++l_row)
            {
                for (uint32_t l_col = 0; l_col < N; ++l_col)
                {
                    this->m_data[l_row][l_col] = f_matrix.m_data[l_row][l_col];
                }
            }
            return *this;
        }

        std::array<T,N>& operator[](uint32_t f_row)
        {
            return m_data[f_row];
        }

        CDataType& operator()(uint32_t f_row, uint32_t f_col)
        {
            return m_data[f_row][f_col];
        }

        const std::array<T,N>& operator[](uint32_t f_row) const
        {
            return m_data[f_row];
        }

        const CDataType& operator()(uint32_t f_row, uint32_t f_col) const
        {
            return m_data[f_row][f_col];
        }

        // template<>
        // CMatrix<T,1,1>::operator CDataType()
        // {
        //     return m_data[0][0];
        // }

        CThisType& operator+() {return *this;}
        CThisType operator-()
        {
            CThisType l_matrix;
            for (uint32_t l_row = 0; l_row < M; ++l_row)
            {
                for (uint32_t l_col = 0; l_col < N; ++l_col)
                {
                    l_matrix.m_data[l_row][l_col] = -this->m_data[l_row][l_col];
                }
            }
            return l_matrix;
        }
        CThisType operator+(const CThisType& f_matrix)
        {
            CThisType l_matrix;
            for (uint32_t l_row = 0; l_row < M; ++l_row)
            {
                for (uint32_t l_col = 0; l_col < N; ++l_col)
                {
                    l_matrix.m_data[l_row][l_col] = this->m_data[l_row][l_col] + f_matrix.m_data[l_row][l_col];
                }
            }
            return l_matrix;
        }
        CThisType operator-(const CThisType& f_matrix)
        {
            CThisType l_matrix;
            for (uint32_t l_row = 0; l_row < M; ++l_row)
            {
                for (uint32_t l_col = 0; l_col < N; ++l_col)
                {
                    l_matrix.m_data[l_row][l_col] = this->m_data[l_row][l_col] - f_matrix.m_data[l_row][l_col];
                }
            }
            return l_matrix;
        }
        CThisType& operator+=(const CThisType& f_matrix)
        {
            for (uint32_t l_row = 0; l_row < M; ++l_row)
            {
                for (uint32_t l_col = 0; l_col < N; ++l_col)
                {
                    this->m_data[l_row][l_col] += f_matrix.m_data[l_row][l_col];
                }
            }
            return *this;
        }
        CThisType& operator-=(const CThisType& f_matrix)
        {
            for (uint32_t l_row = 0; l_row < M; ++l_row)
            {
                for (uint32_t l_col = 0; l_col < N; ++l_col)
                {
                    this->m_data[l_row][l_col] -= f_matrix.m_data[l_row][l_col];
                }
            }
            return *this;
        }

        CThisType operator+(const CDataType& f_val)
        {
            CThisType l_matrix;
            for (uint32_t l_row = 0; l_row < M; ++l_row)
            {
                for (uint32_t l_col = 0; l_col < N; ++l_col)
                {
                    l_matrix.m_data[l_row][l_col] = this->m_data[l_row][l_col] + f_val;
                }
            }
            return l_matrix;
        }
        CThisType operator-(const CDataType& f_val)
        {
            CThisType l_matrix;
            for (uint32_t l_row = 0; l_row < M; ++l_row)
            {
                for (uint32_t l_col = 0; l_col < N; ++l_col)
                {
                    l_matrix.m_data[l_row][l_col] = this->m_data[l_row][l_col] - f_val;
                }
            }
            return l_matrix;
        }
        CThisType& operator+=(const CDataType& f_val)
        {
            for (uint32_t l_row = 0; l_row < M; ++l_row)
            {
                for (uint32_t l_col = 0; l_col < N; ++l_col)
                {
                    this->m_data[l_row][l_col] += f_val;
                }
            }
            return *this;
        }
        CThisType& operator-=(const CDataType& f_val)
        {
            for (uint32_t l_row = 0; l_row < M; ++l_row)
            {
                for (uint32_t l_col = 0; l_col < N; ++l_col)
                {
                    this->m_data[l_row][l_col] -= f_val;
                }
            }
            return *this;
        }
        CThisType& operator*=(const CDataType& f_val)
        {
            for (uint32_t l_row = 0; l_row < M; ++l_row)
            {
                for (uint32_t l_col = 0; l_col < N; ++l_col)
                {
                    this->m_data[l_row][l_col] *= f_val;
                }
            }
            return *this;
        }
        CThisType& operator*=(const CThisType& f_val)
        {
            CThisType& l_thisRef(*this);
            l_thisRef = l_thisRef * f_val;
            return l_thisRef;
        }
        CThisType& operator/=(const CDataType& f_val)
        {
            for (uint32_t l_row = 0; l_row < M; ++l_row)
            {
                for (uint32_t l_col = 0; l_col < N; ++l_col)
                {
                    this->m_data[l_row][l_col] /= f_val;
                }
            }
            return *this;
        }
        CThisType operator*(const CDataType& f_val)
        {
            CThisType l_matrix;
            for (uint32_t l_row = 0; l_row < M; ++l_row)
            {
                for (uint32_t l_col = 0; l_col < N; ++l_col)
                {
                    l_matrix.m_data[l_row][l_col] = this->m_data[l_row][l_col] * f_val;
                }
            }
            return l_matrix;
        }
        template <uint32_t P>
        CRightMultiplicationResultType<P> operator*(const CRightMultipliableType<P>& f_matrix)
        {
            CRightMultiplicationResultType<P> l_matrix;
            for (uint32_t l_row = 0; l_row < M; ++l_row)
            {
                for (uint32_t l_col = 0; l_col < P; ++l_col)
                {
                    l_matrix[l_row][l_col] = 0;
                    for (uint32_t l_idx = 0; l_idx < N; ++l_idx)
                    {
                        l_matrix[l_row][l_col] += this->m_data[l_row][l_idx] * f_matrix[l_idx][l_col];
                    }
                }
            }
            return l_matrix;
        }
        CThisType inv();

        CTransposeType transpose()
        {
            CTransposeType l_trsp;

            for (uint32_t l_row = 0; l_row < M; ++l_row)
            {
                for (uint32_t l_col = 0; l_col < N; ++l_col)
                {
                    l_trsp.m_data[l_col][l_row] = this->m_data[l_row][l_col];
                }
            }

            return l_trsp;
        }

        template <uint32_t P>
        CRightMultiplicationResultType<P> solve(const CRightMultipliableType<P>& f_B)
        {
            return CLUDecomposition(*this).solve();
        }

        static CThisType zeros()
        {
            CThisType l_res;

            for (uint32_t l_row = 0; l_row < M; ++l_row)
            {
                for (uint32_t l_col = 0; l_col < N; ++l_col)
                {
                    l_res[l_row][l_col] = 0;
                }
            }

            return l_res;
        }

        static CThisType ones()
        {
            CThisType l_res;

            for (uint32_t l_row = 0; l_row < M; ++l_row)
            {
                for (uint32_t l_col = 0; l_col < N; ++l_col)
                {
                    l_res[l_row][l_col] = 1;
                }
            }

            return l_res;
        }

        static CThisType eye()
        {
            CThisType l_res(CThisType::zeros());
            uint32_t l_minDim = std::min(M,N);

            for (uint32_t l_row = 0; l_row < l_minDim; ++l_row)
            {
                l_res[l_row][l_row] = 1;
            }

            return l_res;
        }

    protected:
        std::array<std::array<T,N>,M> m_data;
    };

    template<class T, uint32_t N>
    class CLUDecomposition
    {
    public:
        using CThisType = CLUDecomposition<T,N>;
        using COriginalType = CMatrix<T,N,N>;
        using CDataType =T;

        template <uint32_t P>
        using CLeftMultipliableType = CMatrix<T,P,N>;

        template <uint32_t P>
        using CRightMultipliableType = CMatrix<T,N,P>;

        template <uint32_t P>
        using CLeftMultiplicationResultType = CMatrix<T,P,N>;

        template <uint32_t P>
        using CRightMultiplicationResultType = CMatrix<T,N,P>;

        CLUDecomposition(const CThisType& f_decomposition) : m_L(f_decomposition.m_L), m_U(f_decomposition.m_U), m_P(f_decomposition.m_P) {}
        CLUDecomposition(const CThisType&& f_decomposition) : m_L(f_decomposition.m_L), m_U(f_decomposition.m_U), m_P(f_decomposition.m_P) {}
        CLUDecomposition(const COriginalType& f_matrix) : m_L(), m_U(), m_P() {decompose(f_matrix);}
        CLUDecomposition(const COriginalType&& f_matrix) : m_L(), m_U(), m_P() {decompose(f_matrix);}

        operator COriginalType()
        {
            return m_L * m_U;
        }

        COriginalType inv()
        {
            return triUInv() * triLInv();
        }

        COriginalType triLInv()
        {
            COriginalType l_invL(m_L);

            for (uint32_t l_jdx = 0; l_jdx < N; ++l_jdx)
            {
                l_invL[l_jdx][l_jdx] = 1/l_invL[l_jdx][l_jdx];

                for (uint32_t l_idx = l_jdx+1; l_idx < N; ++l_idx)
                {
                    l_invL[l_idx][l_jdx] = 0;

                    for (uint32_t l_kdx = l_jdx; l_kdx < l_idx; ++l_kdx)
                    {
                        l_invL[l_idx][l_jdx] -= m_L[l_idx][l_kdx]*l_invL[l_kdx][l_jdx];
                    }

                    l_invL[l_idx][l_jdx] /= l_invL[l_idx][l_idx];
                }
            }

            return l_invL;
        }

        COriginalType triUInv()
        {
            COriginalType l_invU(m_U);

            for (int32_t l_jdx = (N-1); l_jdx >= 0; --l_jdx)
            {
                l_invU[l_jdx][l_jdx] = 1/l_invU[l_jdx][l_jdx];

                for (int32_t l_idx = (l_jdx-1); l_idx >= 0; --l_idx)
                {
                    l_invU[l_idx][l_jdx] = 0;

                    for (int32_t l_kdx = (l_idx+1); l_kdx < (l_jdx+1); ++l_kdx)
                    {
                        l_invU[l_idx][l_jdx] -= m_U[l_idx][l_kdx]*l_invU[l_kdx][l_jdx];
                    }

                    l_invU[l_idx][l_jdx] /= l_invU[l_idx][l_idx];
                }
            }

            return l_invU;
        }

        template <uint32_t P>
        CRightMultiplicationResultType<P> solve(const CRightMultipliableType<P>& f_B)
        {
            CRightMultipliableType<P> l_B;

            for (uint32_t l_idx = 0; l_idx < N; ++l_idx)
            {
                for (uint32_t l_jdx = 0; l_jdx < P; ++l_jdx)
                {
                    l_B[l_idx][l_jdx] = f_B[m_P[l_idx]][l_jdx];
                }
            }

            return sTriL(sTriU(l_B));
        }

        template <uint32_t P>
        CRightMultiplicationResultType<P> sTriU(const CRightMultipliableType<P>& f_B)
        {
            return triUInv()*f_B;
        }

        template <uint32_t P>
        CRightMultiplicationResultType<P> sTriL(const CRightMultipliableType<P>& f_B)
        {
            // return triLInv()*f_B;
        }


    // private:
        void decompose(const CMatrix<T,N,N>& f_matrix)
                {
            m_U = f_matrix;
            for (uint32_t l_kdx = 0; l_kdx < (N-1); ++l_kdx)
            {
                m_L[l_kdx][l_kdx] = 1;
                m_P[l_kdx] = l_kdx;

                for(uint32_t l_idx = l_kdx+1; l_idx < N; ++l_idx)
                {
                    m_L[l_idx][l_kdx] = m_U[l_idx][l_kdx]/m_U[l_kdx][l_kdx];
                    m_U[l_idx][l_kdx] = 0;
                    for(uint32_t l_jdx = l_kdx+1; l_jdx < N; ++l_jdx)
                    {
                        m_U[l_idx][l_jdx] = m_U[l_idx][l_jdx] - m_L[l_idx][l_kdx]*m_U[l_kdx][l_jdx];
                    }
                }
            }
            m_L[N-1][N-1] = 1;
            m_P[N-1] = N-1;
        }
        CMatrix<T,N,N> m_L;
        CMatrix<T,N,N> m_U;
        std::array<uint32_t,N> m_P;
    };

    template <class T, uint32_t N>
    using CColVector = CMatrix<T,N,1>;

    template <class T, uint32_t N>
    using CVector = CColVector<T,N>;

    template <class T, uint32_t N>
    using CRowVector = CMatrix<T,1,N>;
}; // namespace utils::linalg

#include "linalg.tpp"

#endif // LINALG_H
