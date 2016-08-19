#include "mrd_logger.h"
#include <fstream>
#include <iostream>

void MRDLogger::AddChannel(const std::string &name, const std::string &unit, const void *ptr, LoggerDataType type)
{
  if (channels_.find(name) != channels_.end()) {
    std::cout << name << std::endl;
    throw std::runtime_error("mrdlogger: Duplicate channel name is not allowed.");
  }

  channels_[name].name = name;
  channels_[name].unit = unit;
  channels_[name].type = type;
  channels_[name].channel_id = channels_.size()-1;
  channels_[name].source_ptr = ptr;
  channels_[name].data.resize(max_size_, 0);

  output_order_.push_back(&channels_[name]);
  //std::cout << channels_.size() << " " << output_order_.size() << std::endl;
}

void MRDLogger::ReadFromFile(const std::string &name)
{
  std::ifstream in;
  in.exceptions(std::ifstream::failbit | std::ifstream::badbit);

  size_t n_channels, n_steps, n_data_points;
  float tmp_data;
  char *ptr = (char *)&tmp_data;

  Reset();
  in.open(name);
  // read header
  in >> n_data_points;
  in >> n_channels;
  in >> n_steps;
  in >> frequency_;

  std::vector<std::string> names(n_channels);
  std::vector<std::string> units(n_channels);

  // read the channel names and units
  std::string tmp;
  for (size_t i = 0; i < n_channels; i++) {
    in >> tmp;
    names[i] = tmp;
    in >> tmp;
    units[i] = tmp;
  }

  // get 3 \n
  in.get();
  in.get();
  in.get();

  // read data
  for (size_t t = 0; t < n_steps; t++) {
    bool read_new_data = false;
    for (size_t i = 0; i < n_channels; i++) {
      ptr[3] = in.get();
      ptr[2] = in.get();
      ptr[1] = in.get();
      ptr[0] = in.get();

      auto it = channels_.find(names[i]);
      if (it != channels_.end()) {
        //std::cout << "read: " << it->first << " " << tmp_data << std::endl;
        it->second.data[start_index_] = tmp_data;
        read_new_data = true;
      }
    }

    if (read_new_data) {
      start_index_ = (start_index_ + 1) % max_size_;
      num_data_points_++;
      num_data_points_ = std::min(num_data_points_, max_size_);
    }
    else {
      break;
    }

    //std::cout << "read: " << tmp_data << std::endl;
    //std::cout << "start idx: " << start_index_ << " end idx: " << end_index_ << std::endl;
  }
  in.close();
}

void MRDLogger::WriteToFile(const std::string &name) const
{
  if (size() == 0 || channels_.size() == 0)
    return;

  std::ofstream out;
  out.exceptions(std::ifstream::failbit | std::ifstream::badbit);

  out.open(name.c_str(), std::ofstream::out);
  // write header: total_number_of_data_points number_of_channels number_of_time_steps frequency
  out << this->size()*channels_.size() << " " << channels_.size() << " " << this->size() << " " << frequency_ << std::endl;

  // write names and units
  for (auto it = output_order_.begin(); it != output_order_.end(); it++) {
    out << (*it)->name << " " << (*it)->unit << std::endl;
  }

  // write 2 empty line
  out << std::endl << std::endl;

  // write data
  size_t i = start_index_;
  for (size_t ctr = 0; ctr < num_data_points_; ctr++) {
    for (auto it = output_order_.begin(); it != output_order_.end(); it++) {
      char *ptr = (char *)(&((*it)->data.at(i)));
      out.put(ptr[3]);
      out.put(ptr[2]);
      out.put(ptr[1]);
      out.put(ptr[0]);

      //std::cout << "write: " << (*it)->name << " " << (*it)->data.at(i) << std::endl;
    }
    //std::cout << "write idx: " << i << " end: " << end_index_ << std::endl;

    i = (start_index_+1) % max_size_;
  }

  out.close();
}

void MRDLogger::SaveData()
{
  for (auto it = channels_.begin(); it != channels_.end(); it++) {
    switch (it->second.type) {
      case LOGGER_DATA_TYPE_BOOL:
        it->second.data[start_index_] = (float) *(bool *)(it->second.source_ptr);
        break;
      case LOGGER_DATA_TYPE_CHAR:
        it->second.data[start_index_] = (float) *(char *)(it->second.source_ptr);
        break;
      case LOGGER_DATA_TYPE_INT:
        it->second.data[start_index_] = (float) *(int *)(it->second.source_ptr);
        break;
      case LOGGER_DATA_TYPE_FLOAT:
        it->second.data[start_index_] = (float) *(float *)(it->second.source_ptr);
        break;
      case LOGGER_DATA_TYPE_DOUBLE:
        it->second.data[start_index_] = (float) *(double *)(it->second.source_ptr);
        break;
      case LOGGER_DATA_TYPE_LONG:
        it->second.data[start_index_] = (float) *(long *)(it->second.source_ptr);
        break;
      case LOGGER_DATA_TYPE_UCHAR:
        it->second.data[start_index_] = (float) *(unsigned char *)(it->second.source_ptr);
        break;
      case LOGGER_DATA_TYPE_UINT:
        it->second.data[start_index_] = (float) *(unsigned int *)(it->second.source_ptr);
        break;
      case LOGGER_DATA_TYPE_ULONG:
        it->second.data[start_index_] = (float) *(unsigned long *)(it->second.source_ptr);
        break;

      default:
        continue;
    }
  }

  start_index_ = (start_index_ + 1) % max_size_;
  num_data_points_++;
  num_data_points_ = std::min(num_data_points_, max_size_);

  //std::cout << "start/end " << start_index_ << " " << end_index_ << std::endl;
}

void MRDLogger::PopData()
{
  if (!has_data())
    return;

  for (auto it = channels_.begin(); it != channels_.end(); it++) {
    switch (it->second.type) {
      case LOGGER_DATA_TYPE_BOOL:
        *(bool *)(it->second.source_ptr) = (bool)it->second.data[start_index_];
        break;
      case LOGGER_DATA_TYPE_CHAR:
        *(char *)(it->second.source_ptr) = (char)it->second.data[start_index_];
        break;
      case LOGGER_DATA_TYPE_INT:
        *(int *)(it->second.source_ptr) = (int)it->second.data[start_index_];
        break;
      case LOGGER_DATA_TYPE_FLOAT:
        *(float *)(it->second.source_ptr) = (float)it->second.data[start_index_];
        break;
      case LOGGER_DATA_TYPE_DOUBLE:
        *(double *)(it->second.source_ptr) = (double)it->second.data[start_index_];
        break;
      case LOGGER_DATA_TYPE_LONG:
        *(long *)(it->second.source_ptr) = (long)it->second.data[start_index_];
        break;
      case LOGGER_DATA_TYPE_UCHAR:
        *(unsigned char *)(it->second.source_ptr) = (unsigned char)it->second.data[start_index_];
        break;
      case LOGGER_DATA_TYPE_UINT:
        *(unsigned int *)(it->second.source_ptr) = (unsigned int)it->second.data[start_index_];
        break;
      case LOGGER_DATA_TYPE_ULONG:
        *(unsigned long *)(it->second.source_ptr) = (unsigned long)it->second.data[start_index_];
        break;

      default:
        continue;
    }
  }

  start_index_ = (start_index_+1) % max_size_;
}
