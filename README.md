# DipG-Seg
***DipG-Seg*** is a fast and accurate ground segmentation algorithm based on double images including $z$-image and $d$-image. And this method is pixel-wise, which could be regarded as the counterpart of point-wise on the 3D point cloud. Despite of this, our method is very efficient, meanwhile, accurate.
## 1. Features of DipG-Seg
* A **complete framework of ground segmentation totally based on images**. Thus, it is very easy to further accelerated by adjusting the resolutions of images.
* **Accurate and Super Fast**. DipG-Seg can run at more than 120Hz on a Intel NUC (i7 1165G7) with a resolution of $64\times 870$, achieving a high accuracy of over 94% on the SemanticKITTI dataset.
* **Robust** to LIDAR models and scnenarios. The given parameters allow DipG-Seg to work well on 64, 32, 16-beam LiDARs and in scenarios in nuScenes and SemanticKITTI.

<!-- <p align="center"><img src=./pictures/seq00-gif.gif alt="animated" /></p> -->
<table><tr>
<td><img src= pictures/seq04-gif.gif border=0></td>
<td><img src= pictures/seq08-gif.gif border=0></td>
</tr></table>

## 2. Paper is Available NOW!

Author: [Hao Wen](https://scholar.google.com/citations?user=823HzfIAAAAJ&hl=zh-CN) and [Chunhua Liu](https://scholar.google.com/citations?user=7WEZSaIAAAAJ&hl=zh-CN) from EEPT Lab at CityU.

Paper: [DipG-Seg: Fast and Accurate Double Image-Based Pixel-Wise Ground Segmentation](https://ieeexplore.ieee.org/document/10359455), Hao Wen, Senyi Liu, Yuxin Liu, and Chunhua Liu, T-ITS, Regular Paper
```
@ARTICLE{10359455,
  author={Wen, Hao and Liu, Senyi and Liu, Yuxin and Liu, Chunhua},
  journal={IEEE Transactions on Intelligent Transportation Systems}, 
  title={DipG-Seg: Fast and Accurate Double Image-Based Pixel-Wise Ground Segmentation}, 
  year={2023},
  volume={},
  number={},
  pages={1-12},
  doi={10.1109/TITS.2023.3339334}}

```

Please cite our paper if you find our paper or repo helpful for your research. Thank you!

## 🚀 Exciting Updates Coming Soon! 🚀

We're currently in the process of organizing and refining our codebase to present it in an efficient, readable, and user-friendly manner. Stay tuned for a more polished and accessible code showcase!

Feel free to star ⭐ this repository to stay updated on our progress. We appreciate your interest and patience!
