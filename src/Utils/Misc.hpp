/*
 * Copyright (C) 2023 Erion Plaku
 * All Rights Reserved
 * 
 *   Created by Erion Plaku
 *
 * Code should not be distributed or used without written permission
 * from the copyright holder.
 */


#ifndef GP__MISC_HPP_
#define GP__MISC_HPP_

#include "Utils/PseudoRandom.hpp"
#include "Utils/Algebra.hpp"
#include <vector>
#include <cctype>
#include <cstdlib>

namespace GP
{
    static inline 
    void ToLower(char s[])
    {
	for(int i = 0; s[i] != '\0'; ++i)
	    s[i] = std::tolower(s[i]);
    }
 
    static inline 
    void ToUpper(char s[])
    {
	for(int i = 0; s[i] != '\0'; ++i)
	    s[i] = std::toupper(s[i]);
    }

    static inline
    double StrToDouble(const char s[])
    {
	return strtod(s, NULL);
    }

    static inline
    long int StrToInt(const char s[])
    {
	return strtol(s, NULL, 0);
    }
    
    static inline
    bool StrToBool(const char s[])
    {
	return strcmp(s, "false") != 0;
    }
    
    
    
    template <typename Item>
    static inline Item* AllocArray(const int n)
    {
	return (Item *) calloc(n, sizeof(Item));
    }
    template <typename Item>
    static inline void FreeArray(Item a[])
    {
	if(a)
	    free(a);
    }
    template <typename Item>
    static inline void CopyArray(Item dest[], const int n, const Item src[])
    {
	memcpy(dest, src, n * sizeof(Item));
    }
    
    
    template <typename Item>
    static inline void DeleteItems(const int n, Item items[])
    {
	for(int i = 0; i < n; ++i)
	    if(items[i])
		delete items[i];	
    }
    
    template <typename Item>
    static inline void DeleteItems(std::vector<Item> * const items)
    {
	if(items->size() > 0)
	    DeleteItems<Item>(items->size(), &((*items)[0]));	
    }
    
    template <typename Item>
    static inline int FindItem(const int n, const Item items[], const Item & item)
    {
	for(int i = 0; i < n; ++i)
	    if(items[i] == item)
		return i;
	return -1;
    }
    
    template <typename Item>
    static inline int FindItem(const std::vector<Item> * const items, const Item & item)
    {
	if(items->size() == 0)
	    return -1;
	return FindItem<Item>(items->size(), &((*items)[0]), item);
    }
    
    template <typename Item>
    static inline void ReverseItems(const int n, Item items[])
    {
	Item tmp;	
	for(int i = 0, j = n - 1; i < j; ++i, --j)
	{
	    tmp      = items[i];
	    items[i] = items[j];
	    items[j] = tmp;	    
	}	
    }
    
    template <typename Item>
    static inline void ReverseItems(std::vector<Item> * const items)
    {
	if(items->size() > 0)
	    ReverseItems<Item>(items->size(), &((*items)[0]));	
    }
    
    template <typename Item>
    static void PermuteItems(const int n, Item items[], const int r)
    {
	for(int i = 0; i < r; i++)
	{
	    int  j = RandomUniformInteger(i, n - 1);
	    Item t = items[i]; items[i] = items[j]; items[j] = t;	
	}
    }
    
    template <typename Item>
    static inline void PermuteItems(std::vector<Item> * const items, const int r)
    {
	if(items->size() > 0)
	    PermuteItems<Item>(items->size(), &((*items)[0]), r);
    }

    template <typename Type>
    static inline bool NextPermutation(const int n, Type items[])
    {
	int i, j, k, t;
	
	//starting from right to left examine the array to find
	//the largest tail that is in descending order
	for(i = n - 1; i > 0 && items[i] < items[i-1]; i--);
	
	//the longest tail is the entire permutation
	//i.e.,  n, n-1,..., 2, 1 is the largest possible permutation
	if(i == 0) 
	    return false;
	
	//permutation looks like 
	// items[0] < ... < items[i-2] < items[i-1] < items[i] > items[i+1] > items[i+2] >... > items[n-1]
	
	//since we are after next largest permutation we should keep
	//items[0], items[1],...,items[i-2] unchanged and change items[i-1] to something
	//larger but as small as possible.
	//Need to find items[k] > items[i-1] among the longest tail which is the
	//smallest possible
	for(k = i, j = i+1; j < n; j++)
	    if(items[j] > items[i-1] && items[j] < items[k]) 
		k = j;
	
	//swap items[k] with items[i-1]
	t         = items[k]; 
	items[k]  = items[i-1]; 
	items[i-1]= t;
	
	//reverse tail items[i..n-1]
	k = (n - i) / 2;    
	for(j = 0; j < k; j++)
	{
	    t                = items[i + j]; 
	    items[i+j]       = items[n - 1 - j]; 
	    items[n - 1 - j] = t;
	}
	return true;
    }
    
    static inline
    bool NextCombination(const int n, int combo[], const int r)
    {
	if(r == 0 || combo[0] >= n - r)
	    return false;
	
	int i = r - 1;
	while (combo[i] == n - r + i) 
	    i--;
	
	combo[i] = combo[i] + 1;
	for (int j = i + 1; j < r; j++) 
	    combo[j] = combo[i] + j - i;
	return true;    
    }

    void PointAtDistanceAlongPath(const int n, const double pts[], const double d, const int dim, double p[]);

   void RegularizePointsAlongPath(const int n, 
				  const double pts[], 
				  const double dsep, 
				  const int    dim,
				  std::vector<double> *new_pts);
    
     

    void GenerateGapsU(const int nrGaps,
		       const double gapDurations[],
		       const double duration,		      
		       const double minNonGap,
		       const int nrMustUseTimes,
		       const double mustUseTimes[],
		       std::vector<double> * const gaps);
    
      
    void GenerateGaps(const int nrGaps,
		      const double duration,
		      const double minGap,
		      const double maxGap,
		      const double minNonGap,
		      std::vector<double> * const gaps);

    int NrLinesFile(const char fname[]);

    
    bool NextCounter(const int n, int vals[], const int max[]);
    
}	


#endif



