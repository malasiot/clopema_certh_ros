#include <certh_libs/ApplicationSettings.h>

#include <boost/property_tree/xml_parser.hpp>
#include <boost/foreach.hpp>


using namespace std ;

namespace certh_libs {

ApplicationSettings::ApplicationSettings()
{
}

void ApplicationSettings::load(const std::string &fileName_)
{
    fileName = fileName_ ;
    boost::property_tree::xml_parser::read_xml(fileName, pt, boost::property_tree::xml_parser::trim_whitespace) ;

}

void ApplicationSettings::save(const std::string &fileName_)
{


    // writing the unchanged ptree in file2.xml
    boost::property_tree::xml_writer_settings<char> settings('\t', 1);

    if ( fileName_.empty() )
        boost::property_tree::xml_parser::write_xml(fileName, pt, std::locale(), settings) ;
    else
        boost::property_tree::xml_parser::write_xml(fileName_, pt, std::locale(), settings) ;


}



void ApplicationSettings::linearize(const boost::property_tree::ptree::path_type &childPath, const boost::property_tree::ptree &child, std::multimap<std::string, std::string> &ls)
{
    using boost::property_tree::ptree;

    for( ptree::const_iterator it = child.begin() ; it != child.end() ; ++it)
    {
        std::string data = (*it).second.get_value<std::string>() ;
        if ( !data.empty() ) ls.insert(std::pair<std::string, std::string>(childPath.dump() + '.' + (*it).first, data) )  ;

    }

    for( ptree::const_iterator it = child.begin() ; it !=child.end() ; ++it )
    {
        ptree::path_type curPath = childPath / ptree::path_type(it->first);
        linearize(curPath, it->second, ls);
    }
}

void ApplicationSettings::toMap(std::multimap<std::string, std::string> &ls)
{
    linearize("", pt, ls) ;
}

void ApplicationSettings::keys(const std::string &prefix, std::vector<std::string> &keys) const
{
    using  boost::property_tree::ptree ;

    ptree ch = pt.get_child(prefix) ;

    ptree::const_iterator end = ch.end();
    for (ptree::const_iterator it = ch.begin(); it != end; it++)
    {
        std::string key = (*it).first ;
        std::string val = (*it).second.data() ;

        if ( !val.empty() ) keys.push_back(key) ;
    }
}

void ApplicationSettings::sections(const std::string &prefix, std::vector<std::string> &sections) const
{
    using  boost::property_tree::ptree ;

    ptree ch = pt.get_child(prefix) ;

    ptree::const_iterator end = ch.end();
    for (ptree::const_iterator it = ch.begin(); it != end; it++)
    {
        std::string key = (*it).first ;
        std::string val = (*it).second.data() ;

        if ( val.empty() ) sections.push_back(key) ;
    }
}

void ApplicationSettings::attributes(const std::string &key, std::map<std::string, std::string> &attrs)
{
    using  boost::property_tree::ptree ;

    try {
        ptree ch = pt.get_child(key + ".<xmlattr>") ;

        ptree::const_iterator end = ch.end();
        for (ptree::const_iterator it = ch.begin(); it != end; it++)
        {
            std::string key = (*it).first ;
            std::string val = (*it).second.data() ;

            attrs[key] = val ;
        }
    }
    catch ( boost::property_tree::ptree_bad_path &e )
    {
        return ;
    }

}

void ApplicationSettings::read()
{


}

void ApplicationSettings::write()
{

}

} // namespace certh_libs
