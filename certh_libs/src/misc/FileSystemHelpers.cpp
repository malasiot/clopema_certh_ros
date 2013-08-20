#include <certh_libs/Helpers.h>


#include <boost/filesystem.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>

#include <boost/regex.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/tokenizer.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <boost/lexical_cast.hpp>

#ifdef  _WIN32
#include <windows.h>
#include <Rpc.h>
#pragma comment(lib, "Rpcrt4.lib") // For using UUID generator

#endif


using namespace std ;
using namespace boost::filesystem ;
using namespace boost ;

typedef boost::tokenizer<boost::char_separator<char> >  stokenizer;

namespace certh_libs {

static string globToRegex(const char *pat)
{
    // Convert pattern
    string rx = "(?i)^", be ;

    int i = 0;
    const char *pc = pat ;
    int clen = strlen(pat) ;
    bool inCharClass = false ;

    while (i < clen)
    {
        char c = pc[i++];

        switch (c)
        {
            case '*':
                rx += "[^\\\\/]*" ;
                break;
            case '?':
                rx += "[^\\\\/]" ;
                break;
            case '$':  //Regex special characters
            case '(':
            case ')':
            case '+':
            case '.':
            case '|':
                rx += '\\';
                rx += c;
                break;
            case '\\':
                if ( pc[i] == '*' ) rx += "\\*" ;
                else if ( pc[i] == '?' )  rx += "\\?" ;
                ++i ;
            break ;
            case '[':
            {
                if ( inCharClass )  rx += "\\[";
                else {
                    inCharClass = true ;
                    be += c ;
                }
                break ;
            }
            case ']':
            {
                if ( inCharClass ) {
                    inCharClass = false ;
                    rx += be ;
                    rx += c ;
                    rx += "{1}" ;
                    be.clear() ;
                }
                else rx += "\\]" ;

                break ;
            }
            case '%':
            {
                regex rd("(0\\d)?d") ;
                smatch what;

                if ( regex_match(std::string(pat + i), what, rd,  boost::match_extra) )
                {

                    rx += "[[:digit:]]" ;

                    if ( what.size() == 2 )
                    {
                        rx +=  "{" ;
                        rx += what[1] ;
                        rx += "}" ;
                    }
                    else
                        rx += "+" ;

                    i += what.size() ;
                }
                else
                {
                    if ( inCharClass ) be += c ;
                    else rx += c;
                }
                break ;

            }
            default:
                if ( inCharClass ) be += c ;
                else rx += c;
        }
    }

    rx += "$" ;
    return rx ;
}

static void GlobFindSubDirs(const string &dir, int i, std::vector<string> &pats, std::vector<string> &files, bool trailingSlash)
{
    if ( i > pats.size()-1  ) { // we reached the last element
        if ( is_directory(path(dir))  )
            files.push_back(path(dir + '/').generic_string()) ;
        else if ( !trailingSlash )
            files.push_back(path(dir).generic_string()) ;
        return ;
    }

    regex rm(globToRegex(pats[i].c_str())) ;

    boost::system::error_code ec;
    directory_iterator it(dir, ec) ;

    while (it != directory_iterator())
    {
        path p = (*it).path() ;
        path subpath = p.leaf() ;

        string subpaths = subpath.string() ;

        if ( regex_match(subpaths, rm) )
             GlobFindSubDirs(p.string(), i+1, pats, files, trailingSlash) ;

        it.increment(ec) ;
        if ( ec) ;
     }



}

static regex driverx("(?i)^([a-z]:(?:\\\\|/)|(?:\\\\\\\\|//)[^~!@#$^&()=+\\[\\]{}\\\\|;:',<>/?]+(?:\\\\|/)[^~!@#$^&()=+\\[\\]{}\\\\|;:',<>/?]+(?:[\\\\/])|(?:[\\\\/]))") ;

void glob(const std::string &pattern, std::vector<string> &fnames)
{
    // Find and erase any drive specification

    int matchlen = 0;
    int pos = -1 ;

    smatch what ;
    string dir(pattern) ;
    string drive  ;

    if ( regex_search(dir, what, driverx, boost::match_extra ) )
    {
        pos = 0;

        matchlen = what[0].length() ;

        drive = what[0] ;
        dir = what.suffix() ;
        if ( drive.empty() ) drive = absolute(path(".")).string();
    }
    std::vector<string> el ;

    char_separator<char> sep("/\\", 0, boost::keep_empty_tokens);

    stokenizer tokens(dir, sep);

    std::vector<string> subdirs ;

    bool trailingSlash = false ;

    for (stokenizer::iterator tok_iter = tokens.begin();
         tok_iter != tokens.end(); ++tok_iter )
    {
        string el = *tok_iter ;

        if ( el == "." ) continue ;
        else if ( el == ".." ) {
            if ( subdirs.empty() ) drive += "../" ;
            else subdirs.pop_back() ;
        }
        else if ( el.empty() ) { trailingSlash = true ; continue ; }
        else {
            subdirs.push_back(el) ;
            trailingSlash = false ;
        }

    }

    GlobFindSubDirs(drive, 0, subdirs, fnames, trailingSlash) ;


}

bool pathMatchesFilter(const std::string &filter, const std::string &pathName)
{
    if ( filter.empty() ) return true ;

    std::string sf(filter) ;
    std::vector<string> el ;

    algorithm::split( el, sf, is_any_of(";"), algorithm::token_compress_on );

    for(int i=0 ; i<el.size() ; i++ )
    {
        regex rm(globToRegex(el[i].c_str())) ;

        if ( regex_match(pathName, rm) ) return true ;
    }

    return false ;
}

// get all subdirs that match the filter (sequence of glob patterns)
void subdirs(const std::string &dirName, std::vector<string> &dirs, const std::string &filter)
{
    if ( filter.empty() ) return ;

    std::string sf(filter) ;
    std::vector<string> el ;

    algorithm::split( el, sf, is_any_of(";"), algorithm::token_compress_on );

    boost::system::error_code ec;
    directory_iterator it(dirName, ec), dend ;

    std::vector<regex> pats ;
    for(int i=0 ; i<el.size() ; i++)
        pats.push_back(regex(globToRegex(el[i].c_str()))) ;

    for ( ; it != dend ; ++it )
    {
        path p = (*it) ;
        path pl = p.leaf() ;

        if ( !is_directory(p) ) continue ;

        if ( pats.empty() ) dirs.push_back(pl.string()) ;
        else {
            for(int i=0 ; i<pats.size() ; i++ )
            {
                if ( regex_match(pl.string(), pats[i]) ) { dirs.push_back(pl.string()) ; }
            }
        }

    }

}
bool createDir(const boost::filesystem::path &dir, bool createSubdirs)
{
    //assert(boost::filesystem::is_directory(dir)) ;

    std::string dirPath ;

    boost::filesystem::path::const_iterator it = dir.begin() ;

    // rootpath
    dirPath += (*it++).generic_string() ;
//	dirPath += '/' ;

    while ( it != dir.end() )
    {
        dirPath += (*it++).generic_string() ;
        dirPath += '/' ;

        if ( boost::filesystem::exists(dirPath) ) continue ;

        if ( !createSubdirs && it != dir.end() ) return false ;

        bool res = boost::filesystem::create_directory(dirPath) ;

        if ( !res ) return false ;
    }

    return true ;

}

// get all files in the directory that match the filter (sequence of glob patterns)

void pathEntries(const std::string &pathName, std::vector<string> &files, const std::string &filter)
{
    std::string sf(filter) ;
    std::vector<string> el ;

    algorithm::split( el, sf, is_any_of(";"), algorithm::token_compress_on );

    std::vector<regex> pats ;
    for(int i=0 ; i<el.size() ; i++)
        pats.push_back(regex(globToRegex(el[i].c_str()))) ;

    boost::system::error_code ec;
    directory_iterator it(pathName, ec), dend ;

    for ( ; it != dend ; ++it )
    {
        path p = (*it) ;
        path pl = p.leaf() ;

        if ( is_directory(p) ) continue ;

        if ( pats.empty() ) files.push_back(pl.string()) ;
        else {
            for(int i=0 ; i<pats.size() ; i++ )
            {
                if ( regex_match(pl.string(), pats[i]) ) { files.push_back(pl.string()) ; }
            }
        }

    }
}

string getTemporaryPath(const string &dirName, const string &prefix, const string &ext)
{
#ifdef _WIN32
    string rootPathName ;
	
    if ( dirName == NULL )
    {
        WCHAR tempPath[MAX_PATH] ;
        DWORD len = ::GetTempPathW(MAX_PATH, tempPath) ;

		int rlen = ::WideCharToMultiByte(	CP_UTF8, 0,	tempPath, len,  NULL,   0,  NULL, NULL ) ; //query buffer size

		string buf(rlen, ' ') ;
		int result = ::WideCharToMultiByte(	CP_UTF8, 0,	tempPath, len, 	(LPSTR)(buf.c_str()), rlen,  NULL, NULL ) ;  

        rootPathName = buf ;
		
    }
    else
        rootPathName = dirName ;

    while (1)
    {
        string fileName ;

        UUID uuid;
        UuidCreate ( &uuid );

    
        unsigned char * str;
        UuidToStringA ( &uuid, &str );

        if ( prefix ) fileName += prefix ;
        fileName += string((const char *)str, 6) ;
        fileName += '.' ;
        fileName += (ext) ? ext : "tmp" ;

        string pathName = rootPathName + fileName ;
		
		int rlen = ::MultiByteToWideChar( CP_UTF8, MB_ERR_INVALID_CHARS, pathName.c_str(), 	pathName.length()+1, NULL, 0) ;

		wstring pathNameW(rlen, 0) ;
		
		int result = ::MultiByteToWideChar( CP_UTF8, MB_ERR_INVALID_CHARS, 	pathName.c_str(), 	pathName.length()+1,
			(LPWSTR)pathNameW.c_str(), rlen  ) ;

		if ( result == 0 ) return string() ;

        if ( _waccess(pathNameW.c_str(), 0) == 0 ) continue ;

        return pathName;
    }
	

	return string() ;
#else
    string rootPath ;

    if ( dirName == NULL )
    {
        const char* val = std::getenv("TMPDIR");

        if ( val == 0 ) val = "/tmp/" ;

        path p(val);

        if ( !is_directory(p) ) return string() ;

        rootPath = p.string() ;
    }
    else
        rootPath = dirName ;

    while (1)
    {
        string fileName ;

        boost::uuids::uuid u ;
        std::string s = boost::lexical_cast<std::string>(u);

        if ( !prefix.empty() ) fileName += prefix ;
        fileName += string(s.c_str() + 26, 6) ;
        fileName += '.' ;
        fileName += (!ext.empty()) ? ext : "tmp" ;

        path p(rootPath) ;
        p /= fileName ;

        if ( exists(p) ) continue ;

        return p.string();
    }

#endif
}


bool createDir(const string &dirName, bool createSubdirs)
{
    //assert(boost::filesystem::is_directory(dir)) ;

    boost::filesystem::path dir(dirName) ;

    std::string dirPath ;

    boost::filesystem::path::const_iterator it = dir.begin() ;

    // rootpath
    dirPath += (*it++).generic_string() ;
//	dirPath += '/' ;

    while ( it != dir.end() )
    {
        dirPath += (*it++).generic_string() ;
        dirPath += '/' ;

        if ( boost::filesystem::exists(dirPath) ) continue ;

        if ( !createSubdirs && it != dir.end() ) return false ;

        bool res = boost::filesystem::create_directory(dirPath) ;

        if ( !res ) return false ;
    }

    return true ;

}



} // namespace vl
