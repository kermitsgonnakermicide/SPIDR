#pragma once

#include <cstdint>
#include <vector>

#include <grid_map_msgs/msg/grid_map.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/multi_array_dimension.hpp>

namespace hexapod_nav_cpp {

// Pack a square (N, N) float array as column-major GridMap layer data.
// Matches the Python make_layer() exactly: dim[0]=column_index, dim[1]=row_index
inline std_msgs::msg::Float32MultiArray make_layer(const float *data, int n) {
  std_msgs::msg::Float32MultiArray layer;

  std_msgs::msg::MultiArrayDimension col_dim;
  col_dim.label = "column_index";
  col_dim.size = n;
  col_dim.stride = n * n;

  std_msgs::msg::MultiArrayDimension row_dim;
  row_dim.label = "row_index";
  row_dim.size = n;
  row_dim.stride = n;

  layer.layout.dim.push_back(col_dim);
  layer.layout.dim.push_back(row_dim);

  // Column-major flatten
  layer.data.resize(n * n);
  for (int col = 0; col < n; ++col) {
    for (int row = 0; row < n; ++row) {
      layer.data[col * n + row] = data[row * n + col];
    }
  }

  return layer;
}

// Extract a named layer from a GridMap message as a flat float vector (column-major)
inline bool extract_layer(const grid_map_msgs::msg::GridMap &msg,
                          const std::string &layer_name, std::vector<float> &out,
                          int &size) {
  for (size_t i = 0; i < msg.layers.size(); ++i) {
    if (msg.layers[i] == layer_name) {
      if (i >= msg.data.size()) return false;
      const auto &d = msg.data[i];
      if (d.layout.dim.empty()) return false;
      size = d.layout.dim[0].size;
      out.resize(size * size);
      for (size_t j = 0; j < out.size() && j < d.data.size(); ++j) {
        out[j] = d.data[j];
      }
      return true;
    }
  }
  return false;
}

// Extract cost data from first layer of GridMap (column-major)
inline bool extract_cost_array(const grid_map_msgs::msg::GridMap &msg,
                               std::vector<float> &out, int &size) {
  if (msg.data.empty()) return false;
  const auto &d = msg.data[0];
  if (d.layout.dim.empty()) return false;
  size = d.layout.dim[0].size;
  out.resize(size * size);
  for (size_t j = 0; j < out.size() && j < d.data.size(); ++j) {
    out[j] = d.data[j];
  }
  return true;
}

}  // namespace hexapod_nav_cpp
