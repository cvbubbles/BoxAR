#pragma once


#define _BOXAR_BEG  namespace boxar{
#define _BOXAR_END  }

#ifdef _STATIC_BEG
#undef _STATIC_BEG
#undef _STATIC_END
#endif

#define _STATIC_BEG  using namespace boxar; namespace { 
#define _STATIC_END  }

#define _VX_BEG(x)  namespace boxar{ namespace x{
#define _VX_END()   }}

#define _IMPL_BEG(x) namespace x{
#define _IMPL_END()  }

#ifdef _MSC_VER

#pragma warning(disable:4267)
#pragma warning(disable:4251)
#pragma warning(disable:4190)

#endif


