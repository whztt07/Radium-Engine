#include <Engine/EXE_PATH.hpp>

std::string g_EXE_PATH = "./";
std::string& EXE_PATH()
{
	return g_EXE_PATH;
}
