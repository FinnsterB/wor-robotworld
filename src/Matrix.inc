#include <cassert>
#include <stdexcept>
#include <numeric>
#include <cmath>
#include <utility>
#include <iomanip>

#include "Matrix.hpp"

/**
 *
 */
template< class T, std::size_t M, std::size_t N >
Matrix< T, M, N >::Matrix( T value)
{
	for (std::size_t row = 0; row < M; ++row)
	{
		for (std::size_t column = 0; column < N; ++column)
		{
			matrix.at( row).at( column) = value;
		}
	}
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
Matrix< T, M, N >::Matrix( const std::initializer_list< T >& aList)
{
	// Check the arguments
	assert( aList.size() == M * N);

	auto row_iter = aList.begin();
	for (std::size_t row = 0; row < M; ++row)
	{
		for (std::size_t column = 0; column < N; ++column, ++row_iter)
		{
			matrix.at( row).at( column) = *row_iter;
		}
	}
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
Matrix< T, M, N >::Matrix( const std::initializer_list< std::initializer_list< T > >& aList)
{
	// Check the arguments, the static assert assures that there is at least 1 M and 1 N!
	assert( aList.size() == M && (*aList.begin()).size() == N);

	auto row_iter = aList.begin();
	for (std::size_t row = 0; row < aList.size(); ++row, ++row_iter)
	{
		auto column_iter = (*row_iter).begin();
		for (std::size_t column = 0; column < (*row_iter).size(); ++column, ++column_iter)
		{
			matrix.at( row).at( column) = *column_iter;
		}
	}
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
Matrix< T, M, N >::Matrix( const Matrix< T, M, N >& aMatrix) :
				matrix( aMatrix.matrix)
{
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
std::array< T, N >& Matrix< T, M, N >::at( std::size_t aRowIndex)
{
	return matrix.at( aRowIndex);
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
const std::array< T, N >& Matrix< T, M, N >::at( std::size_t aRowIndex) const
{
	return matrix.at( aRowIndex);
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
T& Matrix< T, M, N >::at( 	std::size_t aRowIndex,
							std::size_t aColumnIndex)
{
	return matrix.at( aRowIndex).at( aColumnIndex);
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
const T& Matrix< T, M, N >::at( std::size_t aRowIndex,
								std::size_t aColumnIndex) const
{
	return matrix.at( aRowIndex).at( aColumnIndex);
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
std::array< T, N >& Matrix< T, M, N >::operator[]( std::size_t aRowIndex)
{
	return matrix[aRowIndex];
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
const std::array< T, N >& Matrix< T, M, N >::operator[]( std::size_t aRowIndex) const
{
	return matrix[aRowIndex];
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
Matrix< T, M, N >& Matrix< T, M, N >::operator=( const Matrix< T, M, N >& rhs)
{
	if (this != &rhs)
	{
		matrix = rhs.matrix;
	}
	return *this;
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
bool Matrix< T, M, N >::operator==( const Matrix< T, M, N >& rhs) const
{
	return matrix == rhs.matrix;
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
template< class T2 >
Matrix< T, M, N >& Matrix< T, M, N >::operator*=( const T2& scalar)
{
	static_assert( std::is_arithmetic<T2>::value, "Value T2 must be arithmetic, see http://en.cppreference.com/w/cpp/types/is_arithmetic");

	for (std::size_t row = 0; row < M; ++row)
	{
		for (std::size_t column = 0; column < N; ++column)
		{
			matrix.at( row).at( column) *= scalar;
		}
	}
	return *this;
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
template< class T2 >
Matrix< T, M, N > Matrix< T, M, N >::operator*( const T2& scalar) const
{
	static_assert( std::is_arithmetic<T2>::value, "Value T2 must be arithmetic, see http://en.cppreference.com/w/cpp/types/is_arithmetic");

	Matrix< T, M, N > result( *this);
	return result *= scalar;
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
template< class T2 >
Matrix< T, M, N >& Matrix< T, M, N >::operator/=( const T2& aScalar)
{
	static_assert( std::is_arithmetic<T2>::value, "Value T2 must be arithmetic, see http://en.cppreference.com/w/cpp/types/is_arithmetic");

	for (std::size_t row = 0; row < M; ++row)
	{
		for (std::size_t column = 0; column < N; ++column)
		{
			matrix.at( row).at( column) /= aScalar;
		}
	}
	return *this;
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
template< class T2 >
Matrix< T, M, N > Matrix< T, M, N >::operator/( const T2& aScalar) const
{
	static_assert( std::is_arithmetic<T2>::value, "Value T2 must be arithmetic, see http://en.cppreference.com/w/cpp/types/is_arithmetic");

	Matrix< T, M, N > result( *this);
	return result /= aScalar;
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
Matrix< T, M, N >& Matrix< T, M, N >::operator+=( const Matrix< T, M, N >& rhs)
{
	for (std::size_t row = 0; row < M; ++row)
	{
		for (std::size_t column = 0; column < N; ++column)
		{
			matrix[row][column] += rhs.at( row, column);
		}
	}
	return *this;
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
Matrix< T, M, N > Matrix< T, M, N >::operator+( const Matrix< T, M, N >& rhs) const
{
	Matrix< T, M, N > result( *this);
	return result += rhs;
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
Matrix< T, M, N >& Matrix< T, M, N >::operator-=( const Matrix< T, M, N >& rhs)
{
	for (std::size_t row = 0; row < M; ++row)
	{
		for (std::size_t column = 0; column < N; ++column)
		{
			matrix[row][column] -= rhs.at( row, column);
		}
	}
	return *this;
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
Matrix< T, M, N > Matrix< T, M, N >::operator-( const Matrix< T, M, N >& rhs) const
{
	Matrix< T, M, N > result( *this);
	return result -= rhs;
}
/**
 * (M, N) * (N, P) -> (M, P)
 */
template< typename T, std::size_t M, std::size_t N >
template< std::size_t columns >
Matrix< T, M, columns > Matrix< T, M, N >::operator*( const Matrix< T, N, columns >& rhs) const
{
	// TODO Implement this function
	Matrix< T, M, columns > result;
	// Check if matrices can be multiplied
    if (this->getColumns() != rhs.getRows()) {
        std::cout << "Error: Matrices cannot be multiplied!" << std::endl;
        return result;
    }

    // Perform matrix multiplication
    for (int i = 0; i < this->getRows(); ++i) {
        for (int j = 0; j < rhs.getColumns(); ++j) {
            for (int k = 0; k < this->getColumns(); ++k) {
                result[i][j] += this->at(i, k)/*matrix[i][k]*/ * rhs.at(k).at(j);
            }
        }
    }
	
	return result;
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
Matrix< T, N, M > Matrix< T, M, N >::transpose() const
{
	Matrix<T, N, M> result;

    //swap rows and columns
    for (std::size_t i = 0; i < M; ++i) {
        for (std::size_t j = 0; j < N; ++j) {
            result[j][i] = (*this)[i][j];
        }
    }

    return result;
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
Matrix< T, M, N > Matrix< T, M, N >::identity() const
{
	// Check if the matrix is square (M == N)
    static_assert(M == N, "Identity matrix can only be created for square matrices");

    Matrix<T, M, N> result;

    // Set the diagonal elements to 1, leaving others as 0
    for (std::size_t i = 0; i < M; ++i) {
        result[i][i] = 1;
    }

    return result;
}
/**
 *
 */
template<class T, std::size_t M, std::size_t N>
Matrix<T, M, N> Matrix<T, M, N>::gauss() const {
    Matrix<T, M, N> result(*this);  // Create a copy of the original
    const double epsilon = 1e-10;  // Small value

    // Perform Gaussian elimination
    for (std::size_t i = 0; i < M; ++i) {
        // Find the pivot row
        std::size_t pivot_row = i;
        for (std::size_t j = i + 1; j < M; ++j) {
            if (std::abs(result[j][i]) > std::abs(result[pivot_row][i])) {
                pivot_row = j;
            }
        }

        // Swoop the current row with the pivot row if needed
        if (pivot_row != i) {
            // Swoop
            for (std::size_t j = 0; j < N; ++j) {
                std::swap(result[i][j], result[pivot_row][j]);
            }
        }

        // Make the diagonal element 1
        T pivot_element = result[i][i];
        if (std::abs(pivot_element) < epsilon) {
            // Handle the case where the pivot element is close to zero
            // This might indicate a singular or nearly singular matrix
            std::cerr << "Warning: Singular or nearly singular matrix." << std::endl;
            continue;
        }

        for (std::size_t j = 0; j < N; ++j) {
            result[i][j] /= pivot_element;
        }

        // Eliminate other rows
        for (std::size_t j = 0; j < M; ++j) {
            if (j != i) {
                T factor = result[j][i];
                for (std::size_t k = 0; k < N; ++k) {
                    result[j][k] -= factor * result[i][k];
                }
            }
        }
    }

    return result;
}

/**
 *
 */

template<class T, std::size_t M, std::size_t N>
Matrix<T, M, N> Matrix<T, M, N>::gaussJordan() const {
    Matrix<T, M, N> result(*this);  // Create a copy of the original
    const double epsilon = 1e-10;  // Small value

    std::size_t lead = 0;

    // Iterate over each row
    for (std::size_t i = 0; i < M; ++i) {
        // Check if we've reached the last column
        if (N <= lead) {
            break;
        }

        std::size_t pivot_row = i;

        // Find the pivot row with a non-zero entry in the current column
        while (std::abs(result[pivot_row][lead]) < epsilon) {
            ++pivot_row;

            // If all rows in the current column are zero, move to the next column
            if (M == pivot_row) {
                pivot_row = i;
                ++lead;

                // If we've reached the last column, break the loop
                if (N == lead) {
                    break;
                }
            }
        }

        // If we've reached the last column, break the outer loop
        if (N == lead) {
            break;
        }

        // Swap the current row with the pivot row if needed
        if (pivot_row != i) {
            for (std::size_t j = 0; j < N; ++j) {
                std::swap(result[i][j], result[pivot_row][j]);
            }
        }

        T pivot_element = result[i][lead];

        // Divide the pivot row by the pivot element to make it 1
        if (std::abs(pivot_element) >= epsilon) {
            for (std::size_t j = 0; j < N; ++j) {
                result[i][j] /= pivot_element;
            }

            // Eliminate other rows
            for (std::size_t j = 0; j < M; ++j) {
                if (j != i) {
                    T factor = result[j][lead];
                    for (std::size_t k = 0; k < N; ++k) {
                        result[j][k] -= factor * result[i][k];
                    }
                }
            }
        }

        ++lead;  // Move to the next column
    }

    return result;
}

template< class T, std::size_t M, std::size_t N >
Matrix< T, M, 1 > Matrix< T, M, N >::solve() const
{
	Matrix<T, M, N> matrix(*this);  // Create a copy of the original

	matrix = matrix.gaussJordan();
    // Extract the solution from the rightmost column of the augmented matrix
    Matrix<T, M, 1> result;
    for (std::size_t i = 0; i < M; ++i) {
        result[i][0] = matrix[i][N-1];
    }

    return result;
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
Matrix< T, M, N > Matrix< T, M, N >::inverse() const
{// Check if the matrix is square (M == N)
    static_assert(M == N, "Inverse can only be computed for square matrices");

    Matrix<T, M, N> augmentedMatrix(*this);  // Copy of the original matrix
    Matrix<T, M, N> result = Matrix<T, M, N>::identity();  // Identity matrix

    const double epsilon = 1e-10;  // Small value to handle floating-point precision issues

    std::size_t lead = 0;

    // Apply Gauss-Jordan elimination to the augmented matrix and identity matrix simultaneously
    for (std::size_t i = 0; i < M; ++i) {
        // Find the pivot row
        std::size_t pivot_row = i;
        while (std::abs(augmentedMatrix[pivot_row][lead]) < epsilon) {
            ++pivot_row;
            if (M == pivot_row) {
                pivot_row = i;
                ++lead;
                if (N == lead) {
                    break;
                }
            }
        }

        // If the pivot element is zero, matrix is singular, and inversion is not possible
        if (std::abs(augmentedMatrix[pivot_row][lead]) < epsilon) {
            throw std::runtime_error("Matrix is singular; cannot compute inverse.");
        }

        // Swap the current row with the pivot row if needed
        if (pivot_row != i) {
            // Directly swap the rows
            for (std::size_t j = 0; j < N; ++j) {
                std::swap(augmentedMatrix[i][j], augmentedMatrix[pivot_row][j]);
                std::swap(result[i][j], result[pivot_row][j]);
            }
        }

        // Make the diagonal element 1
        T pivot_element = augmentedMatrix[i][lead];
        for (std::size_t j = 0; j < N; ++j) {
            augmentedMatrix[i][j] /= pivot_element;
            result[i][j] /= pivot_element;
        }

        // Eliminate other rows
        for (std::size_t j = 0; j < M; ++j) {
            if (j != i) {
                T factor = augmentedMatrix[j][lead];
                for (std::size_t k = 0; k < N; ++k) {
                    augmentedMatrix[j][k] -= factor * augmentedMatrix[i][k];
                    result[j][k] -= factor * result[i][k];
                }
            }
        }

        ++lead;
    }

    return result;
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
std::string Matrix< T, M, N >::to_string() const
{
	std::string result = "Matrix<" + std::to_string( N) + "," + std::to_string( M) + ">\n{\n";
	for (std::size_t i = 0; i < M; ++i)
	{
		for (std::size_t j = 0; j < N; ++j)
		{
			result += std::to_string( matrix[i][j]) + ",";
		}
		result += "\n";
	}
	result += "}";
	return result;
}
/**
 *
 */
template< typename T, const std::size_t N >
bool equals(	const Matrix< T, 1, N >& lhs,
				const Matrix< T, 1, N >& rhs,
				const T aPrecision /*= std::numeric_limits<T>::epsilon()*/,
				const unsigned long aFactor /*= 1*/)
{
	for (std::size_t i = 0; i < N; ++i) {
        if (std::abs(lhs.at(0, i) - rhs.at(0, i)) > aPrecision * static_cast<T>(aFactor)) {
            return false;
        }
    }
    return true;
}
/**
 *
 */
template< typename T, const std::size_t M >
bool equals(	const Matrix< T, M, 1 >& lhs,
				const Matrix< T, M, 1 >& rhs,
				const T aPrecision /*= std::numeric_limits<T>::epsilon()*/,
				const unsigned long aFactor /*= 1*/)
{
	for (std::size_t i = 0; i < M; ++i) {
        if (std::abs(lhs.at(i, 0) - rhs.at(i, 0)) > aPrecision * static_cast<T>(aFactor)) {
            return false;
        }
    }
	return true;
}
/**
 *
 */
template< typename T, const std::size_t M, const std::size_t N >
bool equals(	const Matrix< T, M, N >& lhs,
				const Matrix< T, M, N >& rhs,
				const T aPrecision /*= std::numeric_limits<T>::epsilon()*/,
				const unsigned long aFactor /*= 1*/)
{
	for (std::size_t i = 0; i < M; ++i) {
        for (std::size_t j = 0; j < N; ++j) {
            if (std::abs(lhs.at(i, j) - rhs.at(i, j)) > aPrecision * static_cast<T>(aFactor)) {
                return false;
            }
        }
    }
	return true;
}

template< typename T, std::size_t M, std::size_t N>
std::pair<Matrix<double, M, 1>, Matrix<double, M, M>> Matrix< T, M, N >::doKalmanIteration(Matrix<double, M, 1> stateVector, Matrix<double, M, M> covarianceMatrix, 
																	Matrix<double, M, 1> updateVector, Matrix<double, M, 1> measurementVector,
																	Matrix<double, M, M> naturalChange, Matrix<double, M, M> transformationMatrix,
																	Matrix<double, M, M> observationError)
{	
    std::cout << "Calculating predicted state vector" << std::endl;
	//Predicted state vector calculation
	Matrix<double, M, 1> predictedStateVector = naturalChange * stateVector + transformationMatrix * updateVector;


    std::cout << "Calculating predicted covariance matrix" << std::endl;
	Matrix<double, M, M> predictedCovarianceMatrix = naturalChange * covarianceMatrix * naturalChange.transpose();

    std::cout << "Calculating kalman gain upper" << std::endl;
	Matrix<double, M, M> kalmanGainUpper = (predictedCovarianceMatrix.identity() * predictedCovarianceMatrix * predictedCovarianceMatrix.identity().transpose());
	std::cout << "Calculating kalman gain lower" << std::endl;
    Matrix<double, M, M> kalmanGainLower = (predictedCovarianceMatrix.identity() * predictedCovarianceMatrix * predictedCovarianceMatrix.identity().transpose() + 
						observationError).inverse();

    std::cout << "Calculating kalman gain" << std::endl;
	Matrix<double, M, M> kalmanGain = kalmanGainUpper * kalmanGainLower;

    std::cout << "Calculating observation vector" << std::endl;
	Matrix<double, M, 1> observationVector = /*measurementVector.identity()* */ measurementVector;

    std::cout << "Calculating state vector" << std::endl;
	Matrix<double, M, 1> resultingStateVector = predictedStateVector + kalmanGain * (observationVector - (/*predictedStateVector.identity() * */ predictedStateVector));

    std::cout << "Calculating covariance matrix" << std::endl;
	Matrix<double, M, M> resultingCovarianceMatrix = (kalmanGain.identity() - (kalmanGain * kalmanGain.identity())) * predictedCovarianceMatrix;

	std::pair<Matrix<double, M, 1>, Matrix<double, M, M>> result;
	result.first = resultingStateVector;
	result.second = resultingCovarianceMatrix;
	return result;
}