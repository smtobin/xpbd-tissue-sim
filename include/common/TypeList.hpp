#ifndef __TYPE_LIST_HPP
#define __TYPE_LIST_HPP

#include <type_traits>
#include <variant>

/** Empty struct that stores a parameter pack. Very useful for template metaprogramming. */
template<typename ...Types>
struct TypeList
{
};

/////////////////////////////////////////////////////////////////////////////////////////
// Prepend type to TypeList
/////////////////////////////////////////////////////////////////////////////////////////
template<typename T, typename List>
struct TypeListPrepend;

template<typename T, typename... Types>
struct TypeListPrepend<T, TypeList<Types...>>
{
    using type = TypeList<T, Types...>;
};

/////////////////////////////////////////////////////////////////////////////////////////
// Concatenating TypeLists
//////////////////////////////////////////////////////////////////////////////////////////
// Helper type trait to concatenate TypeLists
template<typename... TypeLists>
struct ConcatenateTypeLists;

// Base case: when no TypeLists are provided
template<>
struct ConcatenateTypeLists<> 
{
    using type = TypeList<>;
};

// Base case: single TypeList
template<typename... Types>
struct ConcatenateTypeLists<TypeList<Types...>> 
{
    using type = TypeList<Types...>;
};

// Recursive case: concatenate multiple TypeLists
template<typename... Types1, typename... Types2, typename... RestLists>
struct ConcatenateTypeLists<TypeList<Types1...>, TypeList<Types2...>, RestLists...> 
{
    using type = typename ConcatenateTypeLists<TypeList<Types1..., Types2...>, RestLists...>::type;
};



///////////////////////////////////////////////////////////////////////////
// Checking if TypeList contains a given type
///////////////////////////////////////////////////////////////////////////
template<typename T, typename List>
struct TypeListContains;

template<typename T, typename... Types>
struct TypeListContains<T, TypeList<Types...>>
{
    static constexpr bool value = (std::is_same_v<T, Types> || ...);
};

// helper metafunction
template <typename T, typename List>
constexpr bool type_list_contains_v = TypeListContains<T, List>::value;


/////////////////////////////////////////////////////////////////////////////
// Removing duplicates from a TypeList
/////////////////////////////////////////////////////////////////////////////
template<typename List>
struct TypeListRemoveDuplicates;

template<>
struct TypeListRemoveDuplicates<TypeList<>>
{
    using type = TypeList<>;
};

template<typename Head, typename... Tail>
struct TypeListRemoveDuplicates<TypeList<Head, Tail...>>
{
    private:

    using tail_deduped = typename TypeListRemoveDuplicates<TypeList<Tail...>>::type;

    public:

    using type = std::conditional_t< TypeListContains<Head, tail_deduped>::value,
        tail_deduped,   // don't include Head if it is already present in tail_deduped
        typename TypeListPrepend<Head, tail_deduped>::type >; // keep Head in the TypeList if not present in tail_deduped
};


///////////////////////////////////////////////////////////////////////////
// Creating std::variant from a TypeList
///////////////////////////////////////////////////////////////////////////
template<typename List>
struct VariantFromTypeList;

template <typename... Types>
struct VariantFromTypeList<TypeList<Types...>>
{
    using type = std::variant<Types...>;
    using ptr_type = std::variant<Types*...>;
    using const_ptr_type = std::variant<const Types*...>;
};



#endif // __TYPE_LIST_HPP