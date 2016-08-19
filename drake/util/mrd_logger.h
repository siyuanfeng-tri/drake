#pragma once

#include <vector>
#include <list>
#include <map>

class MRDLogger {
 protected:
  enum LoggerDataType {
    LOGGER_DATA_TYPE_UNDEF = 0,
    LOGGER_DATA_TYPE_BOOL,
    LOGGER_DATA_TYPE_CHAR,
    LOGGER_DATA_TYPE_INT,
    LOGGER_DATA_TYPE_FLOAT,
    LOGGER_DATA_TYPE_DOUBLE,
    LOGGER_DATA_TYPE_LONG,
    LOGGER_DATA_TYPE_UCHAR,
    LOGGER_DATA_TYPE_UINT,
    LOGGER_DATA_TYPE_ULONG,
    LOGGER_DATA_TYPE_SIZE
  };

  struct DataChannel {
    std::string name;
    std::string unit;
    LoggerDataType type;
    size_t channel_id;
    const void *source_ptr;
    std::vector<float> data;
  };

  float frequency_;

  size_t start_index_;
  size_t num_data_points_;
  const size_t max_size_;

  std::map<const std::string, DataChannel> channels_;
  std::list<const DataChannel *> output_order_;

  void Reset() {
    start_index_ = 0;
    num_data_points_ = 0;
    //channels_.clear();
    //output_order_.clear();
  }

public:
  MRDLogger(size_t max_size) : max_size_(max_size) { Reset(); }
  virtual ~MRDLogger() { ; }
  void SaveData();
  void PopData();

  void ReadFromFile(const std::string &name);
  void WriteToFile(const std::string &name) const;

  void AddChannel(const std::string &name, const std::string &unit, const void *ptr)
    { AddChannel(name, unit, ptr, LOGGER_DATA_TYPE_BOOL); }
  void AddChannel(const std::string &name, const std::string &unit, const char *ptr)
    { AddChannel(name, unit, ptr, LOGGER_DATA_TYPE_CHAR); }
  void AddChannel(const std::string &name, const std::string &unit, const int *ptr)
    { AddChannel(name, unit, ptr, LOGGER_DATA_TYPE_INT); }
  void AddChannel(const std::string &name, const std::string &unit, const float *ptr)
    { AddChannel(name, unit, ptr, LOGGER_DATA_TYPE_FLOAT); }
  void AddChannel(const std::string &name, const std::string &unit, const double *ptr)
    { AddChannel(name, unit, ptr, LOGGER_DATA_TYPE_DOUBLE); }
  void AddChannel(const std::string &name, const std::string &unit, const long *ptr)
    { AddChannel(name, unit, ptr, LOGGER_DATA_TYPE_LONG); }
  void AddChannel(const std::string &name, const std::string &unit, const unsigned char *ptr)
    { AddChannel(name, unit, ptr, LOGGER_DATA_TYPE_UCHAR); }
  void AddChannel(const std::string &name, const std::string &unit, const unsigned int *ptr)
    { AddChannel(name, unit, ptr, LOGGER_DATA_TYPE_UINT); }
  void AddChannel(const std::string &name, const std::string &unit, const unsigned long *ptr)
    { AddChannel(name, unit, ptr, LOGGER_DATA_TYPE_ULONG); }

  inline bool has_data() const { return size() != 0; }
  inline void set_frequency(float f) { frequency_ = f; }

  inline size_t size() const { return num_data_points_; }
  inline size_t max_size() const { return max_size_; }

private:
  void AddChannel(const std::string &name, const std::string &unit, const void *ptr, LoggerDataType type);
};
