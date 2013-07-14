ImageStippling
=======================
image stippling by poisson disk sampling.

The details are [here](http://daiki-yamanaka.hatenablog.com/entry/2013/07/13/163657).

###build

`mkdir build`

`cd build`

`cmake ..`

`make`

###requirement
1.OpenCV
2.Eigen
3.Cairo

###usage
`Usage: Stippling [input_image] [output_images]`

###sample

![input](test/lena.png)
![output](test/lena_stip.png)