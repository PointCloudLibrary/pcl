------------------------------------------------------------------------------------------------
RGB-DI Image Generation Based on Point Cloud Projection Algorithm 
------------------------------------------------------------------------------------------------

Author: Zengshuai Qiu 
		Yan Zhuang
		Fei Yan

Copyright :
      Zengshuai Qiu: ai_qzs@mail.dlut.edu.cn
	  Yan Zhuang: zhuang@dlut.edu.cn
	  Fei Yan: fyan@dlut.edu.cn

Last update :
      20/09/2017

------------------------------------------------------------------------------------------------
Licence
------------------------------------------------------------------------------------------------

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.

The software solves the problem of projecting a three-dimensional point onto a two-dimensional plane 
so as to generate RGB-DI Image according to  (x,y,z,r,g,b,depth,intensity) point cloud collected by UGV.
It is worth noting that the algorithm must be suitable for processing 6D point cloud data, 
rather than confined to data collected by UGV.

Zengshuai Qiu
RGB-DI Image Generating and FCN-Based Outdoor Scene Understanding for UGVs
PhD thesis, School of Control Science and Engineering, Dalian Univ. of Technology, Dalian 116024, China.
 
This software is free ONLY for research purposes. If you want to use any part of the code you
should cite this thesis in any resulting publication. 
The code remains property of Dalian Univ. of Technology.

------------------------------------------------------------------------------------------------
External dependencies
------------------------------------------------------------------------------------------------

This software uses PCL1.7.2 library, Opencv2.4.9 library,Boost1.58 library and OpenMp.

------------------------------------------------------------------------------------------------

------------------------------------------------------------------------------------------------
Usage
------------------------------------------------------------------------------------------------

For convenience, we provide two test data TestData6D_DI0.txt and TestData6D_RGB.txt.
In main.cpp, it is worth noting that for the outdoor scene point cloud, 
if the direction of X axis is upwards,the parameter selection of 0. 
If the direction of Y axis is upwards,the parameter selection of 1.
If the direction of Z axis is upwards,the parameter selection of 2.
------------------------------------------------------------------------------------------------
Installation
------------------------------------------------------------------------------------------------

Current version contains four files for Visual Studio in Windows.
Windows version is self-contained. If there are any problems in other distributions, please
report the problem.

------------------------------------------------------------------------------------------------
Bug reports
------------------------------------------------------------------------------------------------

Please report all bugs to the e-mail address above.
