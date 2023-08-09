use std::ops::{Add, AddAssign, Index, IndexMut, Mul, MulAssign, Neg, Sub, SubAssign};

#[derive(PartialEq, Clone)]
pub struct Matrix {
    data: Vec<Vec<f64>>,
    row: usize,
    col: usize,
}

impl Matrix {
    pub fn zeroes(row: usize, col: usize) -> Self {
        let data = vec![vec![0.; col]; row];

        Self { data, row, col }
    }

    pub fn id(size: usize) -> Self {
        let mut res = Matrix::zeroes(size, size);

        for i in 0..size {
            res[(i, i)] = 1.;
        }

        res
    }

    pub fn from(data: Vec<Vec<f64>>) -> Self {
        let row = data.len();
        let col = data[0].len();

        Self { data, row, col }
    }

    fn map<F: Fn(usize, usize) -> f64>(&self, f: F) -> Self {
        let mut res = Matrix::zeroes(self.row, self.col);
        for x in 0..self.row {
            for y in 0..self.col {
                res[(x, y)] = f(x, y);
            }
        }

        res
    }

    pub fn transpose(&self) -> Self {
        let mut res = Matrix::zeroes(self.col, self.row);

        for x in 0..self.row {
            for y in 0..self.col {
                res[(y, x)] = self[(x, y)];
            }
        }

        res
    }

    pub fn inv(&self) -> Self {
        if self.row != self.col {
            panic!("Matrix is not squared.");
        }

        let n = self.row;

        let mut m = self.clone();

        // println!("Inverting: m: {:#?}", m);

        let mut res = Matrix::id(n);

        // Pass down
        for i in 0..n {
            // println!("For i = {i}:");
            'swap: for k in i..n {
                // println!("  Looking at m[({k},{i})]={} for non zero row.", m[(k, i)]);
                if m[(k, i)] != 0. {
                    let tmp_m = m.data[k].clone();
                    m.data[k] = m.data[i].clone();
                    m.data[i] = tmp_m;

                    let tmp_res = res.data[k].clone();
                    res.data[k] = res.data[i].clone();
                    res.data[i] = tmp_res;

                    // println!("  Swapped {k} & {i}");
                    break 'swap;
                }
            }
            // println!("  After swap: m: {:?}", m);
            // println!("              res: {:?}", res);
            if m[(i, i)] == 0. {
                panic!("Matrix is not invertible.");
            } else {
                let coef = 1. / m[(i, i)];
                for k in 0..n {
                    m[(i, k)] *= coef;
                    res[(i, k)] *= coef;
                }
            }
            // println!("  After dilatation: m: {:?}", m);
            // println!("                    res: {:?}", res);

            for j in (i + 1)..n {
                let coef = m[(j, i)];

                for k in 0..n {
                    m[(j, k)] -= coef * m[(i, k)];
                    res[(j, k)] -= coef * res[(i, k)];
                }
            }

            // println!("  After transvection: m: {:?}", m);
            // println!("                      res: {:?}", res);
        }

        // println!("Pass down, finished.\n");

        // println!("{:#?}", m);

        // Pass up
        for i in (0..n).rev() {
            // println!("For col i = {i}");
            for j in (0..i).rev() {
                // println!("  For row j = {j}");
                let coef = m[(j, i)];
                for k in (0..n).rev() {
                    m[(j, k)] -= coef * m[(i, k)];
                    res[(j, k)] -= coef * res[(i, k)];
                }
            }
        }

        println!("{:#?}", m);

        res
    }
}

impl std::fmt::Debug for Matrix {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        std::fmt::Debug::fmt(&self.data, f)
    }
}

impl Index<(usize, usize)> for Matrix {
    type Output = f64;

    fn index(&self, index: (usize, usize)) -> &Self::Output {
        &self.data[index.0][index.1]
    }
}

impl IndexMut<(usize, usize)> for Matrix {
    fn index_mut(&mut self, index: (usize, usize)) -> &mut Self::Output {
        &mut self.data[index.0][index.1]
    }
}

impl Add<Matrix> for Matrix {
    type Output = Matrix;

    fn add(self, rhs: Matrix) -> Self::Output {
        if self.row != rhs.row {
            panic!("Row size is incompatible.");
        } else if self.col != rhs.col {
            panic!("Col size is incompatible.");
        }

        self.map(|x, y| self[(x, y)] + rhs[(x, y)])
    }
}

impl Add<&Matrix> for &Matrix {
    type Output = Matrix;

    fn add(self, rhs: &Matrix) -> Self::Output {
        if self.row != rhs.row {
            panic!("Row size is incompatible.");
        } else if self.col != rhs.col {
            panic!("Col size is incompatible.");
        }

        self.map(|x, y| self[(x, y)] + rhs[(x, y)])
    }
}

impl AddAssign<Matrix> for Matrix {
    fn add_assign(&mut self, rhs: Matrix) {
        for x in 0..self.row {
            for y in 0..self.col {
                self[(x, y)] += rhs[(x, y)];
            }
        }
    }
}

impl AddAssign<&Matrix> for Matrix {
    fn add_assign(&mut self, rhs: &Matrix) {
        for x in 0..self.row {
            for y in 0..self.col {
                self[(x, y)] += rhs[(x, y)];
            }
        }
    }
}

impl Sub<Matrix> for Matrix {
    type Output = Matrix;

    fn sub(self, rhs: Matrix) -> Self::Output {
        if self.row != rhs.row {
            panic!("Row size is incompatible.");
        } else if self.col != rhs.col {
            panic!("Col size is incompatible.");
        }

        self.map(|x, y| self[(x, y)] - rhs[(x, y)])
    }
}

impl Sub<&Matrix> for &Matrix {
    type Output = Matrix;

    fn sub(self, rhs: &Matrix) -> Self::Output {
        if self.row != rhs.row {
            panic!("Row size is incompatible.");
        } else if self.col != rhs.col {
            panic!("Col size is incompatible.");
        }

        self.map(|x, y| self[(x, y)] - rhs[(x, y)])
    }
}

impl SubAssign<Matrix> for Matrix {
    fn sub_assign(&mut self, rhs: Matrix) {
        for x in 0..self.row {
            for y in 0..self.col {
                self[(x, y)] -= rhs[(x, y)];
            }
        }
    }
}

impl SubAssign<&Matrix> for Matrix {
    fn sub_assign(&mut self, rhs: &Matrix) {
        for x in 0..self.row {
            for y in 0..self.col {
                self[(x, y)] -= rhs[(x, y)];
            }
        }
    }
}

impl Neg for Matrix {
    type Output = Matrix;

    fn neg(self) -> Self::Output {
        self.map(|x, y| -self[(x, y)])
    }
}

impl Neg for &Matrix {
    type Output = Matrix;

    fn neg(self) -> Self::Output {
        self.map(|x, y| -self[(x, y)])
    }
}

impl Mul<Matrix> for Matrix {
    type Output = Matrix;

    fn mul(self, rhs: Matrix) -> Self::Output {
        if self.col != rhs.row {
            panic!(
                "Incompatible sizes: a has {} columns while b has {} rows.",
                self.row, rhs.col
            );
        }

        let mut res = Matrix::zeroes(self.row, rhs.col);

        for x in 0..self.row {
            for y in 0..rhs.col {
                for k in 0..self.col {
                    res[(x, y)] += self[(x, k)] * rhs[(k, y)];
                }
            }
        }

        res
    }
}

impl Mul<&Matrix> for &Matrix {
    type Output = Matrix;

    fn mul(self, rhs: &Matrix) -> Self::Output {
        if self.col != rhs.row {
            panic!(
                "Incompatible sizes: a has {} columns while b has {} rows.",
                self.row, rhs.col
            );
        }

        let mut res = Matrix::zeroes(self.row, rhs.col);

        for x in 0..self.row {
            for y in 0..rhs.col {
                for k in 0..self.col {
                    res[(x, y)] += self[(x, k)] * rhs[(k, y)];
                }
            }
        }

        res
    }
}

impl Mul<f64> for Matrix {
    type Output = Matrix;

    fn mul(self, rhs: f64) -> Self::Output {
        self.map(|x, y| self[(x, y)] * rhs)
    }
}

impl MulAssign<f64> for Matrix {
    fn mul_assign(&mut self, rhs: f64) {
        for x in 0..self.row {
            for y in 0..self.col {
                self[(x, y)] *= rhs;
            }
        }
    }
}

impl Mul<f64> for &Matrix {
    type Output = Matrix;

    fn mul(self, rhs: f64) -> Self::Output {
        self.map(|x, y| self[(x, y)] * rhs)
    }
}

impl Mul<Matrix> for f64 {
    type Output = Matrix;

    fn mul(self, rhs: Matrix) -> Self::Output {
        rhs * self
    }
}

impl Mul<&Matrix> for f64 {
    type Output = Matrix;

    fn mul(self, rhs: &Matrix) -> Self::Output {
        rhs * self
    }
}

#[cfg(test)]
mod tests {
    use crate::linalg::Matrix;

    #[test]
    fn debug() {
        let a = vec![vec![1.7, 5.], vec![5., 4.9]];

        assert_eq!(
            format!("{:?}", Matrix::from(a.clone())),
            format!("{:?}", &a)
        );
    }

    #[test]
    fn mul() {
        let a = Matrix::id(2);
        let b = Matrix::from(vec![vec![4., 9.], vec![2., 7.]]);

        assert_eq!(a * b.clone(), b);

        let a = Matrix::from(vec![vec![1., 2.], vec![3., 4.]]);
        let b = Matrix::from(vec![vec![5.], vec![6.]]);

        let c = Matrix::from(vec![vec![17.], vec![39.]]);

        assert_eq!(a * b, c);
    }

    #[test]
    fn inv() {
        let a = Matrix::from(vec![vec![2., 0., -1.], vec![5., 1., 0.], vec![0., 1., 3.]]);
        let a_inv = Matrix::from(vec![
            vec![3.0, -1.0, 1.0],
            vec![-15.0, 6.0, -5.0],
            vec![5.0, -2.0, 2.0],
        ]);

        assert_eq!(a.inv(), a_inv);

        let b = Matrix::from(vec![vec![0., 0., 1.], vec![0., 1., 0.], vec![1., 0., 0.]]);

        assert_eq!(b.inv(), b);
    }
}
