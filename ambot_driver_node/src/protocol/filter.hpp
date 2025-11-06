#pragma once
// 中值滤波类
class MedianFilter {
private:
    std::vector<float> buffer_;
    size_t window_size_;
    size_t current_index_;
    bool buffer_filled_;
    
public:
    MedianFilter(size_t window_size = 5) 
        : window_size_(window_size), current_index_(0), buffer_filled_(false) {
        buffer_.resize(window_size_, 0.0f);
    }
    
    float filter(float new_value) {
        buffer_[current_index_] = new_value;
        current_index_ = (current_index_ + 1) % window_size_;
        
        if (current_index_ == 0) {
            buffer_filled_ = true;
        }
        
        size_t valid_size = buffer_filled_ ? window_size_ : current_index_;
        if (valid_size == 0) return new_value;
        
        // 创建临时向量进行排序
        std::vector<float> temp_buffer(buffer_.begin(), buffer_.begin() + valid_size);
        std::sort(temp_buffer.begin(), temp_buffer.end());
        
        // 返回中值
        return temp_buffer[valid_size / 2];
    }
    
    void reset() {
        std::fill(buffer_.begin(), buffer_.end(), 0.0f);
        current_index_ = 0;
        buffer_filled_ = false;
    }
};

// 限幅滤波类
class ClipFilter {
private:
    float last_value_ = 0.0f;
    bool initialized_ = false;
    
public:
    float filter(float new_value, float max_change) {
        if (!initialized_) {
            last_value_ = new_value;
            initialized_ = true;
            return new_value;
        }
        
        // 如果变化太大，认为是异常值，使用上次的值
        if (std::abs(new_value - last_value_) > max_change) {
            return last_value_;
        }
        
        last_value_ = new_value;
        return new_value;
    }
    
    void reset() { initialized_ = false; }
};

// 组合滤波类：先中值后限幅
class MedianClipFilter {
private:
    MedianFilter median_filter_;
    ClipFilter clip_filter_;
    float max_change_;
    
public:
    MedianClipFilter(size_t median_window = 3, float max_change = 2.0f)
        : median_filter_(median_window), max_change_(max_change) {}
    
    float filter(float new_value) {
        // 先进行中值滤波去除脉冲噪声
        float  median_value=median_filter_.filter(new_value);
        // 再进行限幅滤波防止突变
        return   clip_filter_.filter(median_value, max_change_);
    }
    
    void reset() {
        median_filter_.reset();
        clip_filter_.reset();
    }
    
    void setMaxChange(float max_change) { max_change_ = max_change; }
};