# MeshFlow_Video_Denoising
Source Code for MeshFlow Video Denoising <br> <br>

OpenCV Version:2.4.11<br><br>

This source code may not have the same processing speed as the paper illustrated. Because I replace the original matching algorithm into a simple one. This may also influence the quality of the denoising result a little. For more project detail, including the academic paper, project abstract and test data, please visiting the project page at http://www.liushuaicheng.org/ICIP/2017/index.html. <br><br>

Usage
-----
1. Create a new folder `build`<br>
2. Inside the folder, using the code below for cmake to build the project files:<br>
```bash
cmake DCMAKE_BUILD_TYPE=Release ..
```
3. Move the test video into this new folder and run this project.<br><br>

For `Visual Studio` users, when run the project file, you should change the `single startup project` option in the solution property into the correct one (not the `ALLBUILD.EXE`).<br>

For Windows users, using `Cmakelists.txt` as the cmakelist. (Remember to change the path for the OpenCV build folder.)<br>
For Linux users, using `Cmakelists_Linux.txt` as the cmakelist.<br><br>

For further questions, feel free to contact me at `albuspeter.rzh@gmail.com`.<br>
Thanks [Guo Heng](https://github.com/GH_HOME) who helped me on the Cmake lists.
