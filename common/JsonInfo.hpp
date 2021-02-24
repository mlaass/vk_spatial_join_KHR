#include <iostream>
#include <string>
#include <fstream>
#include <streambuf>
#include "picojson.h"

class JsonInfo
{
private:
  picojson::object info;
  std::string filename;

public:
  JsonInfo(){}
  JsonInfo(const std::string &filename) : filename(filename)
  {
    load(filename);
  }

  void load(const std::string & fn){
    filename = fn;
    std::ifstream t(filename);
    if (!t.fail())
    {
      //File does not exist code here
      std::string str((std::istreambuf_iterator<char>(t)),
                      std::istreambuf_iterator<char>());
      picojson::value v;
      auto err = picojson::parse(v, str);

      if (!err.empty())
      {
        std::cerr << err << std::endl;
      }

      auto obj = v.get<picojson::object>();
      info.insert(obj.begin(), obj.end());
      //std::cout << info << std::endl;

      t.close();
    }

    add<std::string>("filename", filename, true);
  }

  template <typename T>
  void add(std::string name, T val, bool save_file = false)
  {
    info[name] = picojson::value(val);
    if (save_file)
    {
      save();
    }
  }

  void save()
  {
    std::ofstream file;
    file.open(filename);
    file << picojson::value(info);
    file.close();
  }
  template <typename T>
  T get(const std::string &key)
  {
    return info[key].get<T>();
  }

  bool has(const std::string& key) {
      auto search = info.find(key);
      return search != info.end();
  }
};