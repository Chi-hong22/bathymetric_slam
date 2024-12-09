# Fix Memory Leak in track_error_benchmark

## Description
Fixed a memory leak issue in the track_error_benchmark class that occurred during nested vector destruction of grids data structures.

## Problem
When destructing large nested vector structures containing Eigen matrices in the add_benchmark method, a memory corruption occurred at the vector destructor level. This typically happened with large datasets due to improper memory management of temporary grid maps.

The specific error occurred in:
```cpp
~vector() in libc.so.6!__GI___libc_free() 
```

Affecting the following nested vector structure:

```cpp
vector<vector<vector<Eigen::MatrixXd>>>
```

## Solution
1. Modified add_benchmark() to use smart pointers for managing grid_maps lifecycle
2. Explicitly releasing OpenCV Mats after usage
3. Added better scoping for temporary variables
4. Improved memory management in critical sections

The key changes include:
```cpp
auto grid_maps = std::make_shared<std::vector<std::vector<std::vector<Eigen::MatrixXd>>>>(
    create_grids_from_matrices(maps_points));
```

And explicit resource cleanup:

```cpp
error_img.release();  // Explicit release
mean_img.release();   // Explicit release 
```

```cpp
void track_error_benchmark::add_benchmark(const PointsT& maps_points, const PointsT& tracks_points,
                                        const std::string& name) {
    try {
        
        auto grid_maps = std::make_shared<std::vector<std::vector<std::vector<Eigen::MatrixXd>>>>(
            create_grids_from_matrices(maps_points));

        cv::Mat error_img;
        Eigen::MatrixXd error_vals;
        double consistency_rms_error;
        
        std::tie(consistency_rms_error, error_vals) = compute_consistency_error(*grid_maps);
        error_img = draw_error_consistency_map(error_vals);
        
        std::string error_img_path = dataset_name + "_" + name + "_rms_consistency_error.png";
        if(!error_img.empty()) {
            cv::imwrite(error_img_path, error_img);
            error_img_paths[name] = error_img_path;
            consistency_rms_errors[name] = consistency_rms_error;
            write_matrix_to_file(error_vals, error_img_path);
        }
        error_img.release();  

        std::string mean_img_path = dataset_name + "_" + name + "_mean_depth.png";
        {
            cv::Mat mean_img = draw_height_map(maps_points, mean_img_path);
            if(!mean_img.empty()) {
                cv::imwrite(mean_img_path, mean_img);
            }
            mean_img.release();  
        }

        for (auto hits : {1, 2}) {
            for (auto mean : {true, false}) {
                double average_std;
                Eigen::MatrixXd std_grids;
                
                std::tie(average_std, std_grids) = compute_grid_std(*grid_maps, hits, mean);
                std_metrics[name][hits][mean] = average_std;
                
                std::string std_grids_img_path = dataset_name + "_" + name + "_std_" 
                    + std::to_string(hits) + "_use_mean_" + std::to_string(mean) + ".png";
                
                {
                    cv::Mat grid_img = draw_grid(std_grids);
                    if(!grid_img.empty()) {
                        cv::imwrite(std_grids_img_path, grid_img);
                        write_matrix_to_file(std_grids, std_grids_img_path);
                    }
                    grid_img.release();  
                }
            }
        }

        cout << " -------------- " << endl;
        cout << "Added benchmark " << name << endl;
        cout << "RMS consistency error: " << consistency_rms_error << endl;
        cout << "Consistency image map: " << error_img_path << endl;
        for (auto hits : {1, 2}) {
            for (auto mean : {true, false}) {
                cout << "Std (" << hits << ", use mean = " << mean << "): " 
                     << std_metrics[name][hits][mean] << endl;
            }
        }
        cout << " -------------- " << endl;

    } catch (const std::exception& e) {
        cerr << "Error in add_benchmark: " << e.what() << endl;
        throw;
    }
}
```

