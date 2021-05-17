// adapted from from http://svn.clifford.at/handicraft/2014/polyfit/polyfit.cc

/*
 *  Example code for fitting a polynomial to sample data (using Eigen 3)
 *
 *  Copyright (C) 2014  RIEGL Research ForschungsGmbH
 *  Copyright (C) 2014  Clifford Wolf <clifford@clifford.at>
 *
 *  Permission to use, copy, modify, and/or distribute this software for any
 *  purpose with or without fee is hereby granted, provided that the above
 *  copyright notice and this permission notice appear in all copies.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 *  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 *  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 *  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 *  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 *  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 */

#include <Eigen/QR>
#include <stdio.h>
#include <vector>

////////////////////////////////////////////////////////////////////////////////

template <typename Scalar, size_t N, size_t order>
Eigen::Matrix<Scalar, order+1, 1> polyfit(const Eigen::Matrix<Scalar, N, 1> &xv, const Eigen::Matrix<Scalar, N, 1> &yv)
{
    Eigen::Matrix<Scalar, N, order+1> A;

    EIGEN_STATIC_ASSERT(N >= order+1, THIS_METHOD_IS_ONLY_FOR_VECTORS_OF_A_SPECIFIC_SIZE);

    // create matrix
    for (size_t i = 0; i < N; i++)
    for (size_t j = 0; j < order+1; j++)
        A(i, j) = pow(xv[i], j);

    // solve for linear least squares fit
    return A.householderQr().solve(yv);
}

////////////////////////////////////////////////////////////////////////////////

