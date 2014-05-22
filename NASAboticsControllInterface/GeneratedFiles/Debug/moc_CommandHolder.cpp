/****************************************************************************
** Meta object code from reading C++ file 'CommandHolder.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.2.0)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../CommandHolder.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'CommandHolder.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.2.0. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_CommandHolder_t {
    QByteArrayData data[30];
    char stringdata[461];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    offsetof(qt_meta_stringdata_CommandHolder_t, stringdata) + ofs \
        - idx * sizeof(QByteArrayData) \
    )
static const qt_meta_stringdata_CommandHolder_t qt_meta_stringdata_CommandHolder = {
    {
QT_MOC_LITERAL(0, 0, 13),
QT_MOC_LITERAL(1, 14, 21),
QT_MOC_LITERAL(2, 36, 0),
QT_MOC_LITERAL(3, 37, 5),
QT_MOC_LITERAL(4, 43, 22),
QT_MOC_LITERAL(5, 66, 20),
QT_MOC_LITERAL(6, 87, 21),
QT_MOC_LITERAL(7, 109, 18),
QT_MOC_LITERAL(8, 128, 17),
QT_MOC_LITERAL(9, 146, 13),
QT_MOC_LITERAL(10, 160, 3),
QT_MOC_LITERAL(11, 164, 12),
QT_MOC_LITERAL(12, 177, 4),
QT_MOC_LITERAL(13, 182, 22),
QT_MOC_LITERAL(14, 205, 13),
QT_MOC_LITERAL(15, 219, 8),
QT_MOC_LITERAL(16, 228, 25),
QT_MOC_LITERAL(17, 254, 8),
QT_MOC_LITERAL(18, 263, 26),
QT_MOC_LITERAL(19, 290, 24),
QT_MOC_LITERAL(20, 315, 25),
QT_MOC_LITERAL(21, 341, 21),
QT_MOC_LITERAL(22, 363, 4),
QT_MOC_LITERAL(23, 368, 20),
QT_MOC_LITERAL(24, 389, 16),
QT_MOC_LITERAL(25, 406, 11),
QT_MOC_LITERAL(26, 418, 13),
QT_MOC_LITERAL(27, 432, 8),
QT_MOC_LITERAL(28, 441, 7),
QT_MOC_LITERAL(29, 449, 10)
    },
    "CommandHolder\0FrontLeftWheelChanged\0"
    "\0value\0FrontRightWheelChanged\0"
    "BackLeftWheelChanged\0BackRightWheelChanged\0"
    "BucketPitchChanged\0BucketMineChanged\0"
    "TakingPicture\0pic\0PollingLadar\0poll\0"
    "SwitchingOperationMode\0OperationMode\0"
    "nextMode\0SetFrontLeftWheelVelocity\0"
    "velocity\0SetFrontRightWheelVelocity\0"
    "SetBackLeftWheelVelocity\0"
    "SetBackRightWheelVelocity\0"
    "SetBucketPitchControl\0rate\0"
    "SetBucketMineControl\0SetOperationMode\0"
    "TakePicture\0PollLadarData\0SetFlags\0"
    "bitmask\0ClearFlags\0"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_CommandHolder[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
      20,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       9,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,  114,    2, 0x06,
       4,    1,  117,    2, 0x06,
       5,    1,  120,    2, 0x06,
       6,    1,  123,    2, 0x06,
       7,    1,  126,    2, 0x06,
       8,    1,  129,    2, 0x06,
       9,    1,  132,    2, 0x06,
      11,    1,  135,    2, 0x06,
      13,    1,  138,    2, 0x06,

 // slots: name, argc, parameters, tag, flags
      16,    1,  141,    2, 0x0a,
      18,    1,  144,    2, 0x0a,
      19,    1,  147,    2, 0x0a,
      20,    1,  150,    2, 0x0a,
      21,    1,  153,    2, 0x0a,
      23,    1,  156,    2, 0x0a,
      24,    1,  159,    2, 0x0a,
      25,    0,  162,    2, 0x0a,
      26,    0,  163,    2, 0x0a,
      27,    1,  164,    2, 0x0a,
      29,    1,  167,    2, 0x0a,

 // signals: parameters
    QMetaType::Void, QMetaType::Int,    3,
    QMetaType::Void, QMetaType::Int,    3,
    QMetaType::Void, QMetaType::Int,    3,
    QMetaType::Void, QMetaType::Int,    3,
    QMetaType::Void, QMetaType::Int,    3,
    QMetaType::Void, QMetaType::Int,    3,
    QMetaType::Void, QMetaType::Bool,   10,
    QMetaType::Void, QMetaType::Bool,   12,
    QMetaType::Void, 0x80000000 | 14,   15,

 // slots: parameters
    QMetaType::Void, QMetaType::Int,   17,
    QMetaType::Void, QMetaType::Int,   17,
    QMetaType::Void, QMetaType::Int,   17,
    QMetaType::Void, QMetaType::Int,   17,
    QMetaType::Void, QMetaType::Int,   22,
    QMetaType::Void, QMetaType::Int,   22,
    QMetaType::Void, 0x80000000 | 14,   15,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 28,    2,
    QMetaType::Void, 0x80000000 | 28,    2,

       0        // eod
};

void CommandHolder::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        CommandHolder *_t = static_cast<CommandHolder *>(_o);
        switch (_id) {
        case 0: _t->FrontLeftWheelChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 1: _t->FrontRightWheelChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 2: _t->BackLeftWheelChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 3: _t->BackRightWheelChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 4: _t->BucketPitchChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 5: _t->BucketMineChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 6: _t->TakingPicture((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 7: _t->PollingLadar((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 8: _t->SwitchingOperationMode((*reinterpret_cast< OperationMode(*)>(_a[1]))); break;
        case 9: _t->SetFrontLeftWheelVelocity((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 10: _t->SetFrontRightWheelVelocity((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 11: _t->SetBackLeftWheelVelocity((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 12: _t->SetBackRightWheelVelocity((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 13: _t->SetBucketPitchControl((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 14: _t->SetBucketMineControl((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 15: _t->SetOperationMode((*reinterpret_cast< OperationMode(*)>(_a[1]))); break;
        case 16: _t->TakePicture(); break;
        case 17: _t->PollLadarData(); break;
        case 18: _t->SetFlags((*reinterpret_cast< bitmask(*)>(_a[1]))); break;
        case 19: _t->ClearFlags((*reinterpret_cast< bitmask(*)>(_a[1]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (CommandHolder::*_t)(int );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&CommandHolder::FrontLeftWheelChanged)) {
                *result = 0;
            }
        }
        {
            typedef void (CommandHolder::*_t)(int );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&CommandHolder::FrontRightWheelChanged)) {
                *result = 1;
            }
        }
        {
            typedef void (CommandHolder::*_t)(int );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&CommandHolder::BackLeftWheelChanged)) {
                *result = 2;
            }
        }
        {
            typedef void (CommandHolder::*_t)(int );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&CommandHolder::BackRightWheelChanged)) {
                *result = 3;
            }
        }
        {
            typedef void (CommandHolder::*_t)(int );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&CommandHolder::BucketPitchChanged)) {
                *result = 4;
            }
        }
        {
            typedef void (CommandHolder::*_t)(int );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&CommandHolder::BucketMineChanged)) {
                *result = 5;
            }
        }
        {
            typedef void (CommandHolder::*_t)(bool );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&CommandHolder::TakingPicture)) {
                *result = 6;
            }
        }
        {
            typedef void (CommandHolder::*_t)(bool );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&CommandHolder::PollingLadar)) {
                *result = 7;
            }
        }
        {
            typedef void (CommandHolder::*_t)(OperationMode );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&CommandHolder::SwitchingOperationMode)) {
                *result = 8;
            }
        }
    }
}

const QMetaObject CommandHolder::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_CommandHolder.data,
      qt_meta_data_CommandHolder,  qt_static_metacall, 0, 0}
};


const QMetaObject *CommandHolder::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *CommandHolder::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_CommandHolder.stringdata))
        return static_cast<void*>(const_cast< CommandHolder*>(this));
    return QObject::qt_metacast(_clname);
}

int CommandHolder::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 20)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 20;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 20)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 20;
    }
    return _id;
}

// SIGNAL 0
void CommandHolder::FrontLeftWheelChanged(int _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void CommandHolder::FrontRightWheelChanged(int _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void CommandHolder::BackLeftWheelChanged(int _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void CommandHolder::BackRightWheelChanged(int _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}

// SIGNAL 4
void CommandHolder::BucketPitchChanged(int _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 4, _a);
}

// SIGNAL 5
void CommandHolder::BucketMineChanged(int _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 5, _a);
}

// SIGNAL 6
void CommandHolder::TakingPicture(bool _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 6, _a);
}

// SIGNAL 7
void CommandHolder::PollingLadar(bool _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 7, _a);
}

// SIGNAL 8
void CommandHolder::SwitchingOperationMode(OperationMode _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 8, _a);
}
QT_END_MOC_NAMESPACE
