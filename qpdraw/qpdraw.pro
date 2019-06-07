TEMPLATE = lib

TARGET = qpdraw

SOURCES += \
	src/qpdraw.cpp \
	src/qpdraw_demuxer.cpp \
	src/qpdraw_widget.cpp

HEADERS += \
	include/pdraw/qpdraw.hpp \
	include/pdraw/qpdraw_demuxer.hpp \
	include/pdraw/qpdraw_widget.hpp \
	src/qpdraw_priv.hpp \
	src/qpdraw_demuxer_priv.hpp \
	src/qpdraw_widget_priv.hpp

exists($${OUT_PWD}/alchemy.pri) {
	include($${OUT_PWD}/alchemy.pri)
} else: exists($${PWD}/alchemy.pri) {
	include($${PWD}/alchemy.pri)
}
