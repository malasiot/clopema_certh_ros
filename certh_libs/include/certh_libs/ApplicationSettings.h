#ifndef __APPLICATION_SETTINGS_H__
#define __APPLICATION_SETTINGS_H__

#include <boost/property_tree/ptree.hpp>
#include <map>
#include <vector>

namespace certh_libs {

class ApplicationSettings
{
public:
    ApplicationSettings() ;

    virtual void load(const std::string &fileName) ;
    virtual void save(const std::string &fileName = std::string()) ;

    virtual void read() ;
    virtual void write() ;

    // get all keys corresponding to the given prefix
    void keys(const std::string &prefix, std::vector<std::string> &keys) const ;

    // get all sections corresponding to the given prefix
    void sections(const std::string &prefix, std::vector<std::string> &sections) const ;

    // get attributes of a key
    void attributes(const std::string &key, std::map<std::string, std::string> &attrs) ;

    template <class T>
    T value(const std::string &key, const T &def ) const { return pt.get(key, def); }

    // set the value corresponding to a key with an optional attribute string:
    // bool for boolean,
    // dec:[min,max) or dec:(min,max] or float:(min,max) etc. for numeric values
    // path:filename or path:dir or simply path for specifying that a string is a path.
    // enum:[option1, option2, option3 ..] for an enumeration. The actual value in this case can be a number or a string

    template <class T>
    void set_value(const std::string &key, const T &val, const std::string &options = std::string())  {
        pt.put(key, val);
        if (!options.empty() ) pt.put(key + ".<xmlattr>.options", options) ;
    }

    void toMap(std::multimap<std::string, std::string> &ls) ;

private:

    void set_attribute(const std::string key, const std::string &options) ;
    void linearize(const boost::property_tree::ptree::path_type &childPath, const boost::property_tree::ptree &child, std::multimap<std::string, std::string> &ls) ;

    boost::property_tree::ptree pt ;

    std::string fileName ;


};

}

#endif
