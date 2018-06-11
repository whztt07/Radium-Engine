#ifndef RADIUMENGINE_ATTRIBS_HPP
#define RADIUMENGINE_ATTRIBS_HPP

#include <Core/Containers/VectorArray.hpp>
#include <Core/Log/Log.hpp>
#include <Core/RaCore.hpp>

namespace Ra {
namespace Core {

template <typename T>
class Attrib;

/// AttribBase is the base class for Attrib.
/// An Attrib is data linked to Vertices of a mesh.
/// In the future, it is expected to allow automatic binding between the CPU
/// and the rendered data on the GPU.
class AttribBase {
  public:
    /// attrib name is used to automatic location binding when using shaders.
    virtual ~AttribBase() {}
    std::string getName() const { return m_name; }
    void setName( std::string name ) { m_name = name; }
    virtual void resize( size_t s ) = 0;

    virtual uint getSize() = 0;
    virtual int getStride() = 0;

    bool inline operator==( const AttribBase& rhs ) { return m_name == rhs.getName(); }

    template <typename T>
    inline Attrib<T>& cast() {
        return static_cast<Attrib<T>&>( *this );
    }

    template <typename T>
    inline const Attrib<T>& cast() const {
        return static_cast<const Attrib<T>&>( *this );
    }

    virtual bool isFloat() const = 0;
    virtual bool isVec2() const = 0;
    virtual bool isVec3() const = 0;
    virtual bool isVec4() const = 0;

  private:
    std::string m_name;
};

template <typename T>
class Attrib : public AttribBase {
  public:
    using value_type = T;
    using Container = VectorArray<T>;

    /// resize the container (value_type must have a default ctor).
    void resize( size_t s ) override { m_data.resize( s ); }

    /// RW acces to container data
    inline Container& data() { return m_data; }

    /// R only acccess to container data
    inline const Container& data() const { return m_data; }

    virtual ~Attrib() { m_data.clear(); }
    uint getSize() override { return Container::Matrix::RowsAtCompileTime; }
    int getStride() override { return sizeof( typename Container::value_type ); }

    bool isFloat() const override { return std::is_same<float, T>::value; }
    bool isVec2() const override { return std::is_same<Core::Vector2, T>::value; }
    bool isVec3() const override { return std::is_same<Core::Vector3, T>::value; }
    bool isVec4() const override { return std::is_same<Core::Vector4, T>::value; }

  private:
    Container m_data;
};

template <typename T>
class AttribHandle {
  public:
    typedef T value_type;
    using Container = typename Attrib<T>::Container;

    /// There is no validity check against the corresponding mesh, but just a
    /// simple test to allow the manipuation of unitialized handles.
    constexpr bool isValid() const { return m_idx != -1; }

  private:
    int m_idx = -1;

    friend class AttribManager;
};

/*!
 * \brief The VertexAttribManager provides attributes management by handles
 *
 * The VertexAttribManager stores a container of VertexAttribBase, which can
 * be accessed and deleted (#removeAttrib) using a VertexAttribHandle. Handles
 * are created from an attribute name using #addAttrib, and retrieved using
 * #getAttribHandler.
 *
 * Example of typical use case:
 * \code
 * // somewhere: creation
 * VertexAttribManager mng;
 * auto inputfattrib = mng.addAttrib<float>("MyAttrib");
 *
 * ...
 *
 * // somewhere else: access
 * auto iattribhandler = mng.getAttribHandler<float>("MyAttrib"); //  iattribhandler == inputfattrib
 * if (iattribhandler.isValid()) {
 *    auto &attrib = mng.getAttrib( iattribhandler );
 * }
 * \endcode
 *
 * \warning There is no error check on the handles attribute type
 *
 */
class AttribManager {
  public:
    using value_type = AttribBase*;
    using Container = std::vector<value_type>;

    /// Iterator to iterate over Vector3 Attribs only
    class Vec3Iterator {
        using reference = AttribBase&;

      public:
        Vec3Iterator( Container::const_iterator begin, Container::const_iterator end ) :
            m_current{begin},
            m_end{end} {

            /*            if ( m_current != m_end )
                        {
                            LOG( logINFO ) << "CTR   " << ( *m_current )->getName() << ( *m_current
               )->isVec3()
                                           << "\n";
                        }
                        else
                        { LOG( logINFO ) << "CTR  END\n"; }
            */
            while ( m_current != m_end && !( *m_current )->isVec3() )
            {
                LOG( logINFO ) << "CTR current " << ( *m_current )->getName() << "  "
                               << ( *m_current )->isVec3() << "\n";
                ++m_current;
            }
        }

        Vec3Iterator& operator++() {
            do
            {
                LOG( logINFO ) << "INC current before " << ( *m_current )->getName() << "  "
                               << ( m_current != m_end ) << ( *m_current )->isVec3() << "\n";
                ++m_current;
                LOG( logINFO ) << "INC current after " << ( *m_current )->getName()
                               << ( m_current != m_end ) << ( *m_current )->isVec3() << "\n";
            } while ( m_current != m_end && !( *m_current )->isVec3() );
            return *this;
        }

        bool operator==( const Vec3Iterator& rhs ) { return m_current == rhs.m_current; }
        bool operator!=( const Vec3Iterator& rhs ) { return !( *this == rhs ); }

        reference operator*() const { return **m_current; }

      private:
        Container::const_iterator m_current;
        Container::const_iterator m_end;
    };

    const Container& attribs() const { return m_attribs; }
    /// clear all attribs, invalidate handles !
    void clear() {
        for ( auto a : m_attribs )
        {
            delete a;
        }
        m_attribs.clear();
    }

    Vec3Iterator vec3begin() const {
        Vec3Iterator ret{m_attribs.cbegin(), m_attribs.cend()};
        return ret;
    }

    Vec3Iterator vec3end() const {
        Vec3Iterator ret{m_attribs.cend(), m_attribs.cend()};
        return ret;
    }

    /*!
     * \brief getAttrib Grab an attribute handler by name
     * \param name Name of the attribute
     * \return Attribute handler
     * \code
     * VertexAttribManager mng;
     * auto inputfattrib = mng.addAttrib<float>("MyAttrib");
     *
     * auto iattribhandler = mng.getAttribHandler<float>("MyAttrib"); //  iattribhandler ==
     * inputfattrib if (iattribhandler.isValid()) { // true auto &attrib = mng.getAttrib(
     * iattribhandler );
     * }
     * auto& iattribhandler = mng.getAttribHandler<float>("InvalidAttrib"); // invalid
     * if (iattribhandler.isValid()) { // false
     *    ...
     * }
     * \endcode
     * \warning There is no error check on the attribute type
     */
    template <typename T>
    inline AttribHandle<T> getAttribHandle( const std::string& name ) const {
        auto c = m_attribsIndex.find( name );
        AttribHandle<T> handle;
        if ( c != m_attribsIndex.end() )
        {
            handle.m_idx = c->second;
        }
        return handle;
    }

    /// Get attribute by handle
    template <typename T>
    inline Attrib<T>& getAttrib( AttribHandle<T> h ) {
        return *static_cast<Attrib<T>*>( m_attribs[h.m_idx] );
    }

    /// Get attribute by handle (const)
    template <typename T>
    inline const Attrib<T>& getAttrib( AttribHandle<T> h ) const {
        return *static_cast<Attrib<T>*>( m_attribs[h.m_idx] );
    }

    /// Add attribute by name
    template <typename T>
    AttribHandle<T> addAttrib( const std::string& name ) {
        LOG( logINFO ) << "add attrib " << name << "\n";
        AttribHandle<T> h;
        Attrib<T>* attrib = new Attrib<T>;
        attrib->setName( name );
        m_attribs.push_back( attrib );
        h.m_idx = m_attribs.size() - 1;
        m_attribsIndex[name] = h.m_idx;
        return h;
    }

    /// Remove attribute by name, invalidate all the handles
    void removeAttrib( const std::string& name ) {
        auto c = m_attribsIndex.find( name );
        if ( c != m_attribsIndex.end() )
        {
            int idx = c->second;
            delete m_attribs[idx];
            m_attribs.erase( m_attribs.begin() + idx );
            m_attribsIndex.erase( c );

            // reindex attribs with index superior to removed index
            for ( auto& d : m_attribsIndex )
            {
                if ( d.second > idx )
                {
                    --d.second;
                }
            }
        }
    }

    /// Remove attribute by handle, invalidate all the handles
    template <typename T>
    void removeAttrib( AttribHandle<T> h ) {
        const auto& att = getAttrib( h ); // check the attribute exists
        removeAttrib( att.getName() );
    }

  private:
    std::map<std::string, int> m_attribsIndex;
    Container m_attribs;
};

} // namespace Core
} // namespace Ra

#endif // RADIUMENGINE_ATTRIBS_HPP
