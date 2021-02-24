//
// Created by moritz on 24.02.2021.
//

#ifndef VK_SPATIAL_JOIN_KHR_UTIL_H
#define VK_SPATIAL_JOIN_KHR_UTIL_H

std::string getCmdOption(int argc, char *argv[], const std::string &option, const std::string &def)
{
  std::string cmd = def;
  for (int i = 0; i < argc; ++i)
  {
    std::string arg = argv[i];
    //std::cout << arg << std::endl;
    if (0 == arg.find(option))
    {
      std::size_t l = option.size();
      cmd = arg.substr(l);
      return cmd;
    }
  }
  return cmd;
}

bool getCmdOptionBool(int argc, char *argv[], const std::string &option)
{
  for (int i = 0; i < argc; ++i)
  {
    std::string arg = argv[i];
    if (0 == arg.find(option))
    {
      return true;
    }
  }
  return false;
}


#endif  //VK_SPATIAL_JOIN_KHR_UTIL_H
