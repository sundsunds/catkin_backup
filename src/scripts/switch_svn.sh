#!/bin/bash

pushd `dirname $0`/..

svn switch --relocate svn+ssh://schauwecker@rapc070.cs/home/schauwecker/repository svn+ssh://schauwecker@rapc070.cs.uni-tuebingen.de/home/schauwecker/repository .

popd
