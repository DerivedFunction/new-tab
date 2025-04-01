build:
	colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

output-windows:
	g++ controller/output.cpp -o main.exe -Icontroller/SDL2/include -Lcontroller/SDL2/lib -lmingw32 -lSDL2main -lSDL2

clean:
	rm -f output.txt
	rm -rf build install log
