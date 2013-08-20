#ifndef _HELPERS_H_
#define _HELPERS_H_

#include <vector>
#include <string>
#include <cv.h>

namespace certh_libs {

// File system

// Find all files/dirs matching a filter
// e.g. similar to php Glob function glob("my/*/dir/*.php", files);
// pattern can contain *, ? and character ranges [], [^] as well as %0xd symbols.
// Notice that are ambiguities when backslashes are used (e.g \*) between folder separators
// and glob escapes. Also two question marks followd by slash is a trigraph and therefore should be prefixed by backslash

void glob(const std::string &pattern, std::vector<std::string> &fnames) ;

// tests if filename matches the filter. The filter is a sequence of glob patterns separated by semicolon

bool pathMatchesFilter(const std::string &filter, const std::string &pathName) ;

// get all subdirs that match the filter (sequence of glob patterns)

void subdirs(const std::string &dirName, std::vector<std::string> &dirs, const std::string &filter) ;

// get all files int the directory that match the filter (sequence of glob patterns)

void pathEntries(const std::string &pathName, std::vector<std::string> &files, const std::string &filter) ;

// Get temporary file path located at the specified directory. If dirName is NULL the system temp
// path is used. It does not actually create the file, so it must be used immediately

std::string getTemporaryPath(const std::string &dirName, const std::string &prefix, const std::string &ext) ;

// Create directory. If createSubdirs = true then the full path is created
bool createDir(const std::string &dirName, bool createSubdirs = false) ;

} // namespace cpm
#endif
