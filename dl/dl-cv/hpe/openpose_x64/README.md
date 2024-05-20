# OpenPose docker

Most of the info was gathered from this [link](https://github.com/CMU-Perceptual-Computing-Lab/openpose/blob/master/doc/installation/1_prerequisites.md) 
which states prerequisites for building OpenPose from source. 


# Reinstall CUDNN stuff

Because of reinstall it's neccessary to swap between available cudnn7 installations. 
You can do that with following command: 
```
sudo update-alternatives --config libcudnn
```


# Documentation 

You can find FAQs and all related documentation for openpose [here](https://cmu-perceptual-computing-lab.github.io/openpose/web/html/doc/md_doc_05_faq.html) 

Building OpenPose from source can be found [here](https://github.com/CMU-Perceptual-Computing-Lab/openpose/blob/master/doc/installation/0_index.md#compiling-and-running-openpose-from-source). 


# Out of Memory Error 

If you get following error: 
```
I get an error similar to: Check failed: error == cudaSuccess (2 vs. 0) out of memory.

```

Make sure you have a GPU with at least 4 GB of memory. If your GPU is between 2 and 4 GB, it should be fine for body-only settings, 
but you can also reduce the --net_resolution if it does not fit (check Speed Up, Memory Reduction, and Benchmark for the consequences of this).

# HOWTO start OpenPose with GPU with 2 GB of VRAM? 

You can test OpenPose on weaker GPUs by reducing net resolution and using COCO model. 

This command uses up to 2GB of memory. 

```

 ./build/examples/openpose/openpose.bin --model-pose COCO --net-resolution 256x256 

```

