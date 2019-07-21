#pragma once
#include <Volume.hpp>
#include <iostream>
#include <fstream>
#include <data_types.h>
#include "lookup_tables.hpp"


class MarchingCubes {
public:
	static void extractMesh( Volume& volume, std::string fileName);
};


