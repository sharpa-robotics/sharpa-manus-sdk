
/**
 * @file tensor.h
 * @brief data block and tensor related functionalities
 */

#pragma once

#include <vector>
#include <memory>
#include <cstdint>
#include <stdexcept>
#include <optional>
#include <utility>

namespace sharpa {
namespace tactile {

/**
 * @brief 1D integer range, left-closed, right-open
 */
class Range {
public:
    /**
     * @param l left bound
     * @param r right bound
     */
    Range(size_t l, size_t r);

    /**
     * @return left bound
     */
    size_t l() const;

    /**
     * @return right bound
     */
    size_t r() const;

    /**
     * @param x integer to check
     * @return if integer is inside range
     */
    bool inside(size_t x) const;

    /**
     * @param other another range to compare
     * @return if 2 ranges are exactly the same
     */
    bool operator==(const Range &other) const;

    /**
     * @param other another range to compare
     * @return if 2 ranges are not exactly the same
     */
    bool operator!=(const Range &other) const;
private:
    size_t l_, r_;
};

using Index = std::vector<size_t>;

/**
 * @brief shape of n-Dim data block (tensor)
 */
class Shape {
public:
    /**
     * @param data sizes of each dim
     */
    Shape(std::vector<size_t> data);
    
    /**
     * default constructor, get empty Shape
     */
    Shape();

    /**
     * @return total size. e.g. {4, 6, 1} returns 24
     */
    size_t size() const;

    /**
     * @return begin iterator
     */
    std::vector<size_t>::const_iterator begin() const;

    /**
     * @return end iterator
     */
    std::vector<size_t>::const_iterator end() const;

    /**
     * @return Shape values as std::vector<size_t>
     */
    std::vector<size_t> data() const;

    /**
     * @return dimension of Shape
     */
    size_t dim() const;

    /**
     * @param idx index of same dimension
     * @return 1D index of item if a tensor of given shape is flattened
     * e.g. shape is {1, 2, 3}, idx is {0, 1, 2} flat_idx is 5 (0*2*3+1*3+2)
     */
    size_t flat_idx(const Index &idx) const;

    /**
     * @param idx index of same dimension
     * @return if index is inside of shape, left-closed, right-open
     */
    bool inside(const Index &idx) const;

    /**
     * @param i i-th element of shape
     * @return e.g. shape is {1, 2, 3}, i is 1, return 2
     */
    size_t operator[](size_t i) const;

    /**
     * @param other another shape to compare
     * @return if 2 shapes are exactly the same
     */
    bool operator==(const Shape &other) const;

    /**
     * @param other another shape to compare
     * @return if 2 shapes are not exactly the same
     */
    bool operator!=(const Shape &other) const;

    /**
     * @return e.g. shape is {1, 2, 3} returns {
     *   {0, 0, 0}, {0, 0, 1}, {0, 0, 2},
     *   {0, 1, 0}, {0, 1, 1}, {0, 1, 2},
     * } (6 elements)
     */
    std::vector<Index> all_indices() const;
private:
    std::vector<size_t> data_;
};

/**
 * @brief slice class, for getting a slice of data from a Tensor
 */
class Slice {
public:
    /** constructor */
    Slice();

    /**
     * @param slice list of ranges, size should be same to Tensor to be sliced
     */
    Slice(std::vector<Range> slice);

    /**
     * @return e.g. dimension of slice
     */
    size_t dim() const;

    /**
     * @param idx index to check
     * @return if index is inside of slice
     */
    bool inside(const Index &idx) const;

    /**
     * @return shape of sliced Tensor
     */
    Shape shape() const;

    /**
     * get absolute index from relative index
     * @param idx_rel relative index
     * @return e.g. slice is {{0, 4}, {3, 6}, {1, 3}}, idx_rel is {3, 2, 0}, return {3, 5, 1}
     */
    Index idx_abs(const Index &idx_rel) const;

    /**
     * @param i i-th range of slice
     * @return e.g. slice is {{0, 4}, {3, 6}, {1, 3}}, idx_rel is 1, return {3, 6}
     */
    Range operator[](const size_t i) const;
private:
    std::vector<Range> slice_;
};

template<typename T>
class Tensor;

/**
 * a block of data with specifing value type(int, float, etc.)
 */
class DataBlock {
public:
    /** pointer of DataBlock */
    using Ptr = std::shared_ptr<DataBlock>;

    /**
     * constructor with certain shape, data not initialized
     * @param shape shape of DataBlock
     * @param unit_size e.g. 4 for float, 1 for uint8_t
     */
    DataBlock(Shape shape, size_t unit_size);

    /** copy constructor */
    DataBlock(const DataBlock &other);

    /** move constructor */
    DataBlock(DataBlock &&other);

    /** copy assignment */
    DataBlock& operator=(const DataBlock &other);

    /** move assignment */
    DataBlock& operator=(DataBlock &&other);

    /** deconstructor */
    ~DataBlock();

    /**
     * @return shape of DataBlock
     */
    Shape shape() const;

    /**
     * @return number of elements of DataBlock
     */
    size_t size() const;

    /**
     * @return number of dimensions
     */
    size_t dim() const;

    /**
     * @return number of bytes of DataBlock
     */
    size_t nbytes() const;

    /**
     * initialize all bytes to zero
     */
    void set_zero();

    /**
     * change shape of DataBlock, data are not affected
     * @param shape new shape
     */
    void reshape(Shape shape);

    /**
     * @return create a Tensor of type T (Tensor doesn't own data)
     */
    template<typename T>
    Tensor<T> as() const;

    /**
     * @return const data pointer
     */
    const void *data() const;

    /**
     * @return data pointer
     */
    void *data();

private:
    void *data_;
    Shape shape_;
    size_t unit_size_;
};

/**
 * add 2 DataBlocks together, data are treated as float
 * @param x 1st DataBlock
 * @param y 2nd DataBlock
 * @return sum of 2 DataBlocks
 */
DataBlock::Ptr add_db_float(const DataBlock::Ptr x, const DataBlock::Ptr y);

/**
 * subtract 2 DataBlocks, data are treated as float
 * @param x DataBlock minuend
 * @param y DataBlock subtrahend
 * @return difference of 2 DataBlocks
 */
DataBlock::Ptr sub_db_float(const DataBlock::Ptr x, const DataBlock::Ptr y);

/**
 * muliply(elementwise) DataBlock with a scalor
 * @param x DataBlock
 * @param y scalor
 * @return product DataBlock
 */
DataBlock::Ptr mul_db_float(const DataBlock::Ptr x, float y);

/**
 * cast(elementwise) DataBlock from uint8 to float32
 * @param x uint8 DataBlock
 * @return float32 DataBlock
 */
DataBlock::Ptr db_ui8_to_f32(const DataBlock::Ptr x);

/**
 * cast(elementwise) DataBlock from float32 to uint8
 * @param x float32 DataBlock
 * @return uint8 DataBlock
 */
DataBlock::Ptr db_f32_to_ui8(const DataBlock::Ptr x);

/**
 * load flat float32 DataBlock(1D) from a txt file
 * @param txt_file file path
 * @param size total number of element
 * @return loaded DataBlock
 */
DataBlock::Ptr f32_from_txt(const std::string &txt_file, size_t size);

/**
 * a block of data with specific value type(int, float, etc.)
 * tensor should be generated from DataBlock, and should not own data itself
 */
template<typename T>
class Tensor {
public:
    /**
     * constructor
     * @param data data pointer
     * @param shape_abs absolute shape of Tensor
     * @param slice slice of Tensor, if null, then Tensor has full absolute shape
     */
    Tensor(void *data, Shape shape_abs, std::optional<Slice> slice=std::nullopt)
      : data_((T *)data)
      , shape_abs_(std::move(shape_abs)) {
        if(slice.has_value()) {
            slice_ = *slice;
        } else {
            std::vector<Range> ranges;
            for(size_t i = 0; i < dim(); ++i) ranges.push_back({0, shape_abs_[i]});
            slice_ = Slice(ranges);
        }
    }

    /**
     * @return shape of Tensor (with slicing)
     */
    Shape shape() const { return slice_.shape(); }

    /**
     * @return number of elements (with slicing)
     */
    size_t size() const { return shape().size(); }

    /**
     * @return number of dimensions
     */
    size_t dim() const { return shape_abs_.dim(); }

    /**
     * @return number of bytes (with slicing)
     */
    size_t nbytes() const { return sizeof(T) * size(); }

    /**
     * @return unit_size e.g. 4 for float, 1 for uint8_t
     */
    size_t unit_size() const { return sizeof(T); }

    /**
     * @return e.g. shape is {1, 2, 3} unit_size is 1, return {6, 3, 1}
     */
    std::vector<size_t> stride() const {
        size_t base{unit_size()};
        std::vector<size_t> ret(dim());
        for(int i = dim() - 1; i >= 0; --i) {
            ret[i] = base;
            base *= shape()[i];
        }
        return ret;
    }

    /**
     * get const item on certain index
     * @param index queried index
     * @return item
     */
    const T &at(const Index &index) const {
        Index idx_abs = slice_.idx_abs(index);
        return data_[shape_abs_.flat_idx(idx_abs)];
    }

    /**
     * get item on certain index
     * @param index queried index
     * @return item
     */
    T &at(const Index &index) {
        return const_cast<T&>(std::as_const(*this).at(index));
    }

    /**
     * get a slice Tensor of this Tensor
     * @param s the slice
     */
    Tensor<T> slice(Slice s) const {
        if(s.dim() != dim()) throw std::runtime_error("dim not equal");
        std::vector<Range> s_abs;
        for(size_t i = 0; i < dim(); ++i)
            s_abs.push_back({slice_[i].l() + s[i].l(), slice_[i].l() + s[i].r()});
        return Tensor<T>(data_, shape_abs_, {s_abs});
    }

    /**
     * assign value of another Tensor to this Tensor
     * @param other the other Tensor
     */
    void assign(const Tensor<T> &other) {
        if(shape() != other.shape()) throw std::runtime_error("shape not equal");
        /* TO_DO to be optimized */
        for(const auto &ids : shape().all_indices()) {
            at(ids) = other.at(ids);
        }
    }

    /**
     * @return const data pointer
     */
    const T *data() const { return data_; }

    /**
     * @return data pointer
     */
    T *data() { return data_; }
    
private:
    T *data_;
    Shape shape_abs_;
    Slice slice_;
};

template<typename T>
Tensor<T> DataBlock::as() const {
    if(sizeof(T) != unit_size_) throw std::runtime_error("unit size dismatch");
    return Tensor<T>(data_, shape_);
}

}
}
