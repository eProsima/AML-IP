// Copyright 2022 Proyectos y Sistemas de Mantenimiento SL (eProsima).
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*!
 * @file TemplatesDataType.hpp
 */

#include <fastcdr/config.h>

#ifndef AMLIPCPP_TYPES_TEMPLATESDATATYPE_HPP
#define AMLIPCPP_TYPES_TEMPLATESDATATYPE_HPP

#include <cpp_utils/utils.hpp>

#include <types/multiservice/MsReferenceDataType.hpp>
#include <amlip_cpp/types/InterfaceDataType.hpp>


namespace eprosima {
namespace amlip {
namespace types {

/**
 * TODO
 */
template <typename T>
class MsDataType : public MsReferenceDataType
{

    FORCE_TEMPLATE_SUBCLASS(types::InterfaceDataType, T);

public:

    MsDataType();

    MsDataType(
            const AmlipIdDataType& client_id,
            const TaskId& task_id,
            const AmlipIdDataType& server_id,
            const T& data);

    MsDataType(
            const MsReferenceDataType& reference,
            const T& data);

    MsDataType(
            MsReferenceDataType&& reference,
            T&& data);

    /////
    // InterfaceDataType methods

    /**
     * @brief Whether the type is bounded
     *
     * @warning this method must be overriden in child class.
     */
    static bool is_bounded();

    /**
     * @brief Whether the type is plain
     *
     * @warning this method must be overriden in child class.
     */
    static bool is_plain();

    /**
     * @brief Construct an empty sample in the memory allocated
     *
     * @pre The type must be plain
     *
     * @param memory already allocated memory for the new data
     *
     * @return true if the construction was successful, false otherwise
     *
     * @warning this method must be overriden in child class.
     */
    static bool construct_sample(
            void* memory);

    /**
     * @brief Name of the Data Type. This name will be used as the DDS type name.
     *
     * @warning this method must be overriden in child class.
     */
    static std::string type_name();

    const T& data() const;

    T& data();

    void data(
            T new_value);

    static uint32_t max_cdr_typesize_;

    static constexpr uint32_t max_key_cdr_typesize_ {0UL};

protected:

    static const char* TYPE_NAME_;

    T data_;
};


template <typename T>
class RpcRequestDataType : public InterfaceDataType
{

    FORCE_TEMPLATE_SUBCLASS(types::InterfaceDataType, T);

public:

    /**
     * @brief Default constructor.
     */
    RpcRequestDataType();

    /**
     * @brief Construct a new RpcRequestDataType object.
     *
     * @param client_id Id of the Participant (associated with the RPC Client)
     * @param task_id Id of the task
     * @param server_id Id of the Participant (associated with the RPC Server)
     * @param
     *
     */
    RpcRequestDataType(
            const AmlipIdDataType& client_id,
            const TaskId& task_id,
            const AmlipIdDataType& server_id,
            const T& data);

    /*!
     * @brief Default destructor.
     */
    virtual ~RpcRequestDataType();

    /*!
     * @brief Copy constructor.
     * @param x Reference to the object RpcRequestDataType that will be copied.
     */
    RpcRequestDataType(
            const RpcRequestDataType& x);

    /*!
     * @brief Move constructor.
     * @param x Reference to the object RpcRequestDataType that will be copied.
     */
    RpcRequestDataType(
            RpcRequestDataType&& x);

    /*!
     * @brief Copy assignment.
     * @param x Reference to the object RpcRequestDataType that will be copied.
     */
    RpcRequestDataType& operator =(
            const RpcRequestDataType& x);

    /*!
     * @brief Move assignment.
     * @param x Reference to the object RpcRequestDataType that will be copied.
     */
    RpcRequestDataType& operator =(
            RpcRequestDataType&& x);

    /*!
     * @brief Comparison operator.
     * @param x RpcRequestDataType object to compare.
     */
    bool operator ==(
            const RpcRequestDataType& x) const;

    /*!
     * @brief Comparison operator.
     * @param x RpcRequestDataType object to compare.
     */
    bool operator !=(
            const RpcRequestDataType& x) const;

    /*!
     * @brief Comparison operator.
     * @param x RpcRequestDataType object to compare.
     */
    bool operator <(
            const RpcRequestDataType& x) const;

    /*!
     * @brief Return value of attribute \c client_id_
     */
    AmlipIdDataType client_id() const;

    /*!
     * @brief Returns reference of attribute \c client_id_
     */
    AmlipIdDataType& client_id();

    /*!
     * @brief This function copies the value in member \c client_id_
     * @param new_value New value to be copied in member id \c client_id_
     */
    void client_id(
            const AmlipIdDataType& new_value);

    /*!
     * @brief Return value of attribute \c task_id_
     */
    TaskId task_id() const;

    /*!
     * @brief Returns reference of attribute \c task_id_
     */
    TaskId& task_id();

    /*!
     * @brief This function copies the value in member \c task_id_
     * @param new_value New value to be copied in member id \c task_id_
     */
    void task_id(
            const TaskId& new_value);

    /*!
     * @brief Returns value of attribute \c server_id_
     */
    AmlipIdDataType server_id() const;

    /*!
     * @brief Returns reference of attribute \c server_id_
     */
    AmlipIdDataType& server_id();

    /*!
     * @brief This function copies the value in member \c server_id_
     * @param new_value New value to be copied in member id \c server_id_
     */
    void server_id(
            const AmlipIdDataType& new_value);

    /////
    // InterfaceDataType methods

    /**
     * @brief Whether the type is bounded
     *
     * @warning this method must be overriden in child class.
     */
    static bool is_bounded();

    /**
     * @brief Whether the type is plain
     *
     * @warning this method must be overriden in child class.
     */
    static bool is_plain();

    /**
     * @brief Construct an empty sample in the memory allocated
     *
     * @pre The type must be plain
     *
     * @param memory already allocated memory for the new data
     *
     * @return true if the construction was successful, false otherwise
     *
     * @warning this method must be overriden in child class.
     */
    static bool construct_sample(
            void* memory);

    /**
     * @brief Name of the Data Type. This name will be used as the DDS type name.
     *
     * @warning this method must be overriden in child class.
     */
    static std::string type_name();

    /*!
     * @brief Return value of attribute \c data_
     */
    const T& data() const;

    T& data();

    /*!
     * @brief This function copies the value in member \c data_
     * @param new_value New value to be copied in member id \c data_
     */
    void data(
            T new_value);

    static uint32_t max_cdr_typesize_;

    static constexpr uint32_t max_key_cdr_typesize_ {0UL};

protected:

    AmlipIdDataType client_id_;

    TaskId task_id_;

    AmlipIdDataType server_id_;

    static const char* TYPE_NAME_; // "rpc_request"

    T data_;

};

//! \c RpcRequestDataType to stream serializator
template <typename T>
std::ostream& operator <<(
        std::ostream& os,
        const RpcRequestDataType<T>& request);


template <typename T>
class RpcReplyDataType : public InterfaceDataType
{

    FORCE_TEMPLATE_SUBCLASS(types::InterfaceDataType, T);

public:

    /**
     * @brief Default constructor.
     */
    RpcReplyDataType();

    /**
     * @brief Construct a new RpcReplyDataType object.
     *
     * @param client_id Id of the Participant (associated with the RPC Client)
     * @param task_id Id of the task
     * @param server_id Id of the Participant (associated with the RPC Server)
     * @param data
     *
     */
    RpcReplyDataType(
            const AmlipIdDataType& client_id,
            const TaskId& task_id,
            const AmlipIdDataType& server_id,
            const T& data);

    /*!
     * @brief Default destructor.
     */
    virtual ~RpcReplyDataType();

    /*!
     * @brief Copy constructor.
     * @param x Reference to the object RpcReplyDataType that will be copied.
     */
    RpcReplyDataType(
            const RpcReplyDataType& x);

    /*!
     * @brief Move constructor.
     * @param x Reference to the object RpcReplyDataType that will be copied.
     */
    RpcReplyDataType(
            RpcReplyDataType&& x);

    /*!
     * @brief Copy assignment.
     * @param x Reference to the object RpcReplyDataType that will be copied.
     */
    RpcReplyDataType& operator =(
            const RpcReplyDataType& x);

    /*!
     * @brief Move assignment.
     * @param x Reference to the object RpcReplyDataType that will be copied.
     */
    RpcReplyDataType& operator =(
            RpcReplyDataType&& x);

    /*!
     * @brief Comparison operator.
     * @param x RpcReplyDataType object to compare.
     */
    bool operator ==(
            const RpcReplyDataType& x) const;

    /*!
     * @brief Comparison operator.
     * @param x RpcReplyDataType object to compare.
     */
    bool operator !=(
            const RpcReplyDataType& x) const;

    /*!
     * @brief Comparison operator.
     * @param x RpcReplyDataType object to compare.
     */
    bool operator <(
            const RpcReplyDataType& x) const;

    /*!
     * @brief Returns value of attribute \c client_id_
     */
    AmlipIdDataType client_id() const;

    /*!
     * @brief Returns reference of attribute \c client_id_
     */
    AmlipIdDataType& client_id();

    /*!
     * @brief This function copies the value in member \c client_id_
     * @param new_value New value to be copied in member id \c client_id_
     */
    void client_id(
            const AmlipIdDataType& new_value);

    /*!
     * @brief Returns value of attribute \c task_id_
     */
    TaskId task_id() const;

    /*!
     * @brief Returns reference of attribute \c task_id_
     */
    TaskId& task_id();

    /*!
     * @brief This function copies the value in member \c task_id_
     * @param new_value New value to be copied in member id \c task_id_
     */
    void task_id(
            const TaskId& new_value);

    /*!
     * @brief Returns value of attribute \c server_id_
     */
    AmlipIdDataType server_id() const;

    /*!
     * @brief Returns reference of attribute \c server_id_
     */
    AmlipIdDataType& server_id();

    /*!
     * @brief This function copies the value in member \c server_id_
     * @param new_value New value to be copied in member id \c server_id_
     */
    void server_id(
            const AmlipIdDataType& new_value);

    /////
    // InterfaceDataType methods

    /**
     * @brief Whether the type is bounded
     *
     * @warning this method must be overriden in child class.
     */
    static bool is_bounded();

    /**
     * @brief Whether the type is plain
     *
     * @warning this method must be overriden in child class.
     */
    static bool is_plain();

    /**
     * @brief Construct an empty sample in the memory allocated
     *
     * @pre The type must be plain
     *
     * @param memory already allocated memory for the new data
     *
     * @return true if the construction was successful, false otherwise
     *
     * @warning this method must be overriden in child class.
     */
    static bool construct_sample(
            void* memory);

    /**
     * @brief Name of the Data Type. This name will be used as the DDS type name.
     *
     * @warning this method must be overriden in child class.
     */
    static std::string type_name();

    /*!
     * @brief Return value of attribute \c data_
     */
    const T& data() const;

    T& data();

    /*!
     * @brief This function copies the value in member \c data_
     * @param new_value New value to be copied in member id \c data_
     */
    void data(
            T new_value);

    static uint32_t max_cdr_typesize_;

    static constexpr uint32_t max_key_cdr_typesize_ {0UL};

protected:

    AmlipIdDataType client_id_;

    TaskId task_id_;

    AmlipIdDataType server_id_;

    static const char* TYPE_NAME_;

    T data_;
};

//! \c RpcReplyDataType to stream serializator
template <typename T>
std::ostream& operator <<(
        std::ostream& os,
        const RpcReplyDataType<T>& reply);

} /* namespace types */
} /* namespace amlip */
} /* namespace eprosima */

// Include auxiliary functions like for serializing/deserializing.
#include <types/impl/TemplatesDataTypeCdrAux.ipp>

#include <types/impl/TemplatesDataType.ipp>

#endif // AMLIPCPP_TYPES_TEMPLATESDATATYPE_HPP
