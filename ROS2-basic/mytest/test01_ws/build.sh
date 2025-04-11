export PYTHONWARNINGS="ignore"
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

#colcon build \
#	--cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=Debug \
#	--package-select demo_cpp_pkg

