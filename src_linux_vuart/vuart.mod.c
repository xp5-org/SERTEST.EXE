#include <linux/module.h>
#include <linux/export-internal.h>
#include <linux/compiler.h>

MODULE_INFO(name, KBUILD_MODNAME);

__visible struct module __this_module
__section(".gnu.linkonce.this_module") = {
	.name = KBUILD_MODNAME,
	.init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
	.exit = cleanup_module,
#endif
	.arch = MODULE_ARCH_INIT,
};



static const struct modversion_info ____versions[]
__used __section("__versions") = {
	{ 0xbd03ed67, "__ref_stack_chk_guard" },
	{ 0xdc54cc96, "__platform_driver_register" },
	{ 0xac5fb72f, "platform_device_register_full" },
	{ 0x7a4a235c, "__tty_alloc_driver" },
	{ 0xf4fa66aa, "tty_std_termios" },
	{ 0x92c5f7d6, "tty_port_init" },
	{ 0x73e445fa, "tty_port_link_device" },
	{ 0x99b8bcd2, "tty_register_driver" },
	{ 0x12c15411, "tty_register_device" },
	{ 0xe8213e80, "_printk" },
	{ 0xd272d446, "__stack_chk_fail" },
	{ 0x86891b00, "__tty_insert_flip_string_flags" },
	{ 0x894786aa, "tty_flip_buffer_push" },
	{ 0x7851be11, "__get_user_1" },
	{ 0xd272d446, "__fentry__" },
	{ 0xd272d446, "__x86_return_thunk" },
	{ 0x90a48d82, "__ubsan_handle_out_of_bounds" },
	{ 0x7d8e36f3, "tty_port_close" },
	{ 0x151cfdba, "tty_port_open" },
	{ 0xaa5303d9, "tty_unregister_device" },
	{ 0x9ff29899, "tty_unregister_driver" },
	{ 0x9ff29899, "tty_driver_kref_put" },
	{ 0x92c5f7d6, "tty_port_destroy" },
	{ 0x5e6d74fc, "platform_device_unregister" },
	{ 0xb620cc7d, "platform_driver_unregister" },
	{ 0xe4de56b4, "__ubsan_handle_load_invalid_value" },
	{ 0xe9196a28, "module_layout" },
};

static const u32 ____version_ext_crcs[]
__used __section("__version_ext_crcs") = {
	0xbd03ed67,
	0xdc54cc96,
	0xac5fb72f,
	0x7a4a235c,
	0xf4fa66aa,
	0x92c5f7d6,
	0x73e445fa,
	0x99b8bcd2,
	0x12c15411,
	0xe8213e80,
	0xd272d446,
	0x86891b00,
	0x894786aa,
	0x7851be11,
	0xd272d446,
	0xd272d446,
	0x90a48d82,
	0x7d8e36f3,
	0x151cfdba,
	0xaa5303d9,
	0x9ff29899,
	0x9ff29899,
	0x92c5f7d6,
	0x5e6d74fc,
	0xb620cc7d,
	0xe4de56b4,
	0xe9196a28,
};
static const char ____version_ext_names[]
__used __section("__version_ext_names") =
	"__ref_stack_chk_guard\0"
	"__platform_driver_register\0"
	"platform_device_register_full\0"
	"__tty_alloc_driver\0"
	"tty_std_termios\0"
	"tty_port_init\0"
	"tty_port_link_device\0"
	"tty_register_driver\0"
	"tty_register_device\0"
	"_printk\0"
	"__stack_chk_fail\0"
	"__tty_insert_flip_string_flags\0"
	"tty_flip_buffer_push\0"
	"__get_user_1\0"
	"__fentry__\0"
	"__x86_return_thunk\0"
	"__ubsan_handle_out_of_bounds\0"
	"tty_port_close\0"
	"tty_port_open\0"
	"tty_unregister_device\0"
	"tty_unregister_driver\0"
	"tty_driver_kref_put\0"
	"tty_port_destroy\0"
	"platform_device_unregister\0"
	"platform_driver_unregister\0"
	"__ubsan_handle_load_invalid_value\0"
	"module_layout\0"
;

MODULE_INFO(depends, "");


MODULE_INFO(srcversion, "48D1B094FAFEBE44BB7F2D0");
